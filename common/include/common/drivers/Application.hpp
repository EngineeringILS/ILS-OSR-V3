#ifndef LUNABOTICS_APPLICATION_HPP_
#define LUNABOTICS_APPLICATION_HPP_

#include <cstdint>

namespace Lunabotics {
namespace Common {
namespace Applications {

/**
 * @brief Common lifecycle interface for executable applications.
 *
 * An application coordinates platform services and hardware drivers.
 * run() may block until stop() is called or the application exits normally.
 */
class Application {
public:
    enum class ApplicationState : uint8_t {
        UNINITIALIZED,
        INITIALIZED,
        RUNNING,
        STOPPING,
        STOPPED,
        ERROR
    };

    // TODO: Move HostController into a global common definition shared by
    // applications and peripheral interfaces.
    enum class HostController : uint8_t {
        OBC,
        ESP32
    };

    explicit Application(HostController host) : host_(host) {}
    virtual ~Application() = default;

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    /**
     * @brief Acquires and initializes resources required by the application.
     */
    virtual bool init() = 0;

    /**
     * @brief Runs the application until stopped or an error occurs.
     */
    virtual void run() = 0;

    /**
     * @brief Requests that the application stop running.
     */
    virtual void stop() = 0;

    /**
     * @brief Releases resources owned by the application.
     */
    virtual void deinit() = 0;

    ApplicationState getState() const {
        return state_;
    }

    HostController getHost() const {
        return host_;
    }

protected:
    void setState(ApplicationState state) {
        state_ = state;
    }

private:
    ApplicationState state_ = ApplicationState::UNINITIALIZED;
    const HostController host_;
};

} // namespace Applications
} // namespace Common
} // namespace Lunabotics

#endif
