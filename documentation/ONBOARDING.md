# Project Development Guide

## 1. Development Philosophy
We adhere to sensible Software Engineering regimes designed to maintain high code quality and sanity. This boils down to three core pillars:

* **DRY (Don't Repeat Yourself):** Avoidance of duplicate efforts and redundant code.
* **Measure Twice, Cut Once:** Rigorous planning and testing before implementation.
* **Collaborative Quality** Regular code reviews (Peer-to-Peer and Lead-to-Member) for all major pull requests.

> **Note:** For deep dives on syntax and standards, refer to the **[Contribution Documentation](./ONBOARDING.md)**.

---

## 2. Repository Stucture: The Hybrid Monorepo
We utilize a **Hybrid Monorepo** approach. This allows us to host multiple platforms with different compilation and environment requirements in a single location.

**The Core Directories:**

* [common/](./../common) → **C++ Shared Library:** The core platform-independent development environment for shared C++ code (Hardware Abstraction of Motors/Sensors and key communication libraries).
* [python/](./../python) → **Python Environment** Primarily frontend control interface development and testing using DearPyGui.
* [firmware/](./../firmware) → **ESP-IDF Environment:** Embedded systems and microcontroller code.
* [ros2_ws/](./../ros2_ws/) → **ROS2 Environment:** Robot Operating System workspaces.

> **Note:** *Specific setup instructions are WIP, but are currently in the root [README.md](./../README.md)*

---

## 3. What This Means For you
The ultimate goal is to help you develop **industry-transferrable software skills**

**The Process:**

1.  **Workshops:** We will hold a workshop over the break covering Git fundamentals (branching, pulling, committing) so everyone starts on the same page.
2.  **Task Assignment:** I will begin assigning small, manageable programming projects to get you acclimated to the repo.
3.  **Code Reviews:** We will utilize a simple code review process.
4.  **Time Management:** We aim to tackle coding during our weekly scheduled time to minimize the impact on your personal time.