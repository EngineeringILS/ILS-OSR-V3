import math
import os
import select
import subprocess
import threading
import time
import unittest

from imu_serial import Ack, DeviceError, IMUClient, ImuSample, ProtocolError, parse_line


def sample(request_id=0, sequence=1):
    return f"IMU,1,{request_id},{sequence},9007199254740993,1.25,0,0,0,0,-0.5,0,42,0,25\n".encode()


class FakeSerial:
    def __init__(self, data=b'', respond=None, write_size=3):
        self.data = bytearray(data)
        self.sent = bytearray()
        self.pending = bytearray()
        self.respond = respond
        self.write_size = write_size
        self.timeout = 17
        self.write_timeout = 19
        self.closed = False

    def read(self, size):
        data = bytes(self.data[:size])
        del self.data[:size]
        return data

    def write(self, data):
        count = min(self.write_size, len(data))
        self.sent.extend(data[:count])
        self.pending.extend(data[:count])
        while b'\n' in self.pending:
            line, _, rest = self.pending.partition(b'\n')
            self.pending[:] = rest
            if self.respond:
                self.data.extend(self.respond(bytes(line)))
        return count

    def close(self):
        self.closed = True


def acknowledge(line):
    _, request, action = line.split(b',')
    return b'ACK,' + request + b',' + action + b'\n'


class ClientTests(unittest.TestCase):
    def make_client(self, transport, **kwargs):
        client = IMUClient(transport=transport, timeout=0.03, **kwargs)
        self.addCleanup(client.close)
        return client

    def test_parse_units_and_integer_precision(self):
        record = parse_line(sample(7, 4294967296).decode())
        self.assertEqual(record.timestamp_us, 9007199254740993)
        self.assertEqual(record.sequence, 4294967296)
        self.assertEqual(record.acceleration_m_s2, (1.25, 0, 0))
        self.assertEqual(record.angular_velocity_rad_s, (0, 0, -0.5))
        self.assertEqual(record.magnetic_field_ut, (0, 42, 0))
        self.assertEqual(record.temperature_c, 25)
        self.assertEqual(parse_line('ACK,1,STOP\r\n'), Ack(1, 'STOP'))
        self.assertIsNone(parse_line('ESP boot log'))

    def test_malformed_records_rejected(self):
        valid = sample().decode().strip().split(',')
        invalid = ['IMU,2,0', 'ACK,0,STOP', 'ACK,x,STOP', 'ACK,1,READ',
                   'ERR', 'ERR,-1,FAIL', 'ERR,1,', 'ERR,1,a b', 'ACK,1,STOP,extra']
        for index, value in [(1, '2'), (2, '4294967296'), (2, '-1'), (2, '+1'),
                             (3, str(2**64)), (4, str(2**63)), (4, '1.5'),
                             (5, 'nan'), (6, 'inf'), (9, 'oops')]:
            fields = valid.copy()
            fields[index] = value
            invalid.append(','.join(fields))
        for line in invalid:
            with self.subTest(line=line), self.assertRaises(ProtocolError):
                parse_line(line)

    def test_boot_noise_coalesced_records_and_ownership(self):
        transport = FakeSerial(b'boot\xff\n' + sample(sequence=1) + sample(sequence=2))
        with self.make_client(transport) as client:
            self.assertEqual(client.read_sample().sequence, 1)
            self.assertEqual(client.read_sample().sequence, 2)
            self.assertEqual(transport.timeout, 17)
        self.assertTrue(transport.closed)
        with self.assertRaises(OSError):
            client.read_sample()

    def test_partial_line_survives_timeout(self):
        wire = sample()
        transport = FakeSerial(wire[:17])
        client = self.make_client(transport)
        with self.assertRaises(TimeoutError):
            client.read_sample()
        self.assertEqual(transport.timeout, 17)
        transport.data.extend(wire[17:])
        self.assertEqual(client.read_sample().sequence, 1)

    def test_oversized_line_discard_and_recovery(self):
        transport = FakeSerial(b'x' * 513)
        client = self.make_client(transport)
        with self.assertRaises(ProtocolError):
            client.read_sample()
        with self.assertRaises(TimeoutError):
            client.read_sample()
        # A valid-looking suffix of the bad frame must not become a sample.
        transport.data.extend(sample(sequence=99) + sample(sequence=2))
        self.assertEqual(client.read_sample().sequence, 2)

    def test_short_writes_interleaving_and_queue_overflow(self):
        transport = FakeSerial(respond=lambda line: sample(sequence=1) + sample(sequence=2)
                               + sample(sequence=3) + acknowledge(line))
        client = self.make_client(transport, queue_size=2)
        self.assertEqual(client.start(), Ack(1, 'START'))
        self.assertEqual(transport.sent, b'CMD,1,START\n')
        self.assertEqual(transport.write_timeout, 19)
        self.assertEqual(client.dropped_samples, 1)
        self.assertEqual(client.read_sample().sequence, 2)
        self.assertEqual(client.read_sample().sequence, 3)
        client.stop()
        self.assertFalse(client._samples)

    def test_read_once_errors_and_no_stale_response(self):
        def respond(line):
            _, request, _ = line.split(b',')
            return b'ACK,88,START\n' + sample(88) + sample(0) + sample(int(request), 9)
        client = self.make_client(FakeSerial(respond=respond))
        self.assertEqual(client.read_once().sequence, 9)
        self.assertEqual(client.read_sample().request_id, 0)
        client._io.serial.respond = lambda line: b'ERR,2,ESP_ERR_NOT_FINISHED\n'
        with self.assertRaises(DeviceError) as error:
            client.read_once()
        self.assertEqual(error.exception.code, 'ESP_ERR_NOT_FINISHED')

    def test_async_failure_and_init_recovery(self):
        transport = FakeSerial(b'ERR,0,ESP_ERR_NOT_FOUND\n', respond=acknowledge)
        client = self.make_client(transport)
        with self.assertRaises(DeviceError):
            client.read_sample()
        transport.data.extend(b'ERR,0,ESP_ERR_NOT_FOUND\n')
        self.assertEqual(client.reinitialize(), Ack(1, 'INIT'))
        self.assertEqual(client.last_device_error.code, 'ESP_ERR_NOT_FOUND')
        self.assertEqual(transport.sent, b'CMD,1,INIT\n')

    def test_timeout_does_not_allow_stale_ack_to_complete_next_command(self):
        transport = FakeSerial()
        client = self.make_client(transport)
        with self.assertRaises(TimeoutError):
            client.start()
        transport.data.extend(b'ACK,1,START\n')
        transport.respond = acknowledge
        self.assertEqual(client.stop(), Ack(2, 'STOP'))

    def test_write_failures_close_connection(self):
        for count in [None, 0, -1, 500]:
            with self.subTest(count=count):
                transport = FakeSerial()
                transport.write = lambda data: count
                client = self.make_client(transport)
                with self.assertRaises(OSError):
                    client.start()
                self.assertTrue(transport.closed)
                self.assertEqual(transport.write_timeout, 19)

    def test_validation_and_ids_never_wrap(self):
        for timeout in [0, -1, math.nan, math.inf]:
            with self.assertRaises(ValueError):
                IMUClient(transport=FakeSerial(), timeout=timeout)
        transport = FakeSerial(respond=acknowledge)
        client = self.make_client(transport)
        with self.assertRaises(ValueError):
            client.command('START\nCMD,2,STOP')
        self.assertFalse(transport.sent)
        client._next_id = 2**32 - 1
        self.assertEqual(client.start().request_id, 2**32 - 1)
        with self.assertRaises(OverflowError):
            client.start()

    def test_wrong_response_type_or_action(self):
        for response in [b'ACK,1,STOP\n', sample(1)]:
            client = self.make_client(FakeSerial(respond=lambda _: response))
            with self.assertRaises(ProtocolError):
                client.start()

    @unittest.skipUnless(os.environ.get('IMU_PROTOCOL_TEST_EXECUTABLE'), 'C++ fixture path not set')
    def test_actual_cpp_formatter_output(self):
        line = subprocess.check_output([os.environ['IMU_PROTOCOL_TEST_EXECUTABLE'], '--sample'], text=True)
        self.assertEqual(parse_line(line), parse_line(sample(7, 4294967296).decode()))

    @unittest.skipUnless(os.name == 'posix', 'requires a POSIX pseudo-terminal')
    def test_pyserial_pseudoterminal_roundtrip(self):
        import serial  # Exercise the installed dependency, not a serial mock.
        master, slave = os.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        finished = threading.Event()
        commands = []
        errors = []

        def device():
            pending = b''
            try:
                while not finished.is_set():
                    if not select.select([master], [], [], 0.05)[0]:
                        continue
                    pending += os.read(master, 512)
                    while b'\n' in pending:
                        command, pending = pending.split(b'\n', 1)
                        commands.append(command)
                        _, request, action = command.split(b',')
                        response = sample(int(request)) if action == b'READ' else acknowledge(command)
                        if action == b'START':
                            response += sample()
                        os.write(master, response[:5])
                        os.write(master, response[5:])
            except Exception as exc:
                errors.append(exc)

        with IMUClient(os.ttyname(slave), timeout=1.0) as client:
            thread = threading.Thread(target=device, daemon=True)
            thread.start()
            try:
                client.stop()
                client.start()
                self.assertIsInstance(client.read_sample(), ImuSample)
                self.assertEqual(client.read_once().request_id, 3)
                client.reinitialize()
                client.stop()
            finally:
                finished.set()
                thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors)
        self.assertEqual(commands, [b'CMD,1,STOP', b'CMD,2,START', b'CMD,3,READ',
                                    b'CMD,4,INIT', b'CMD,5,STOP'])


if __name__ == '__main__':
    unittest.main()
