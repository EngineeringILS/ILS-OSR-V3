# Standards and Conventions

This is a list of standards and conventions that we follow in the different Lunabotics Codebases we work on. These standards are designed to ensure consistency, readability, and maintainability of the codebase. The codebases will be converted or will be made to adhere to the standards and conventions listed below. These are not meant to be exhaustive, but rather a simple enough set of rules that can be easily followed by all team/subteam members and contributors without much overhead. The goal is predictability, consistency, and maintainability but not rigidigty that discourages creativity, innovation, or participation.

## Table of Contents
**Features, Commits, and Pull Requests**
- [Commit Messages](#commit-messages)
- [Branching Strategy](#branching-strategy)
- [Hotfix Branches](#hotfix-branches)
- [Merge Conflict Resolution](#merge-conflict-resolution)
- [Pull Request Guidelines](#pull-request-guidelines)
- [Continuous Integration and Deployment](#continuous-integration-and-deployment)
**Release Cycle and Versioning**
- [Release Strategy](#release-strategy)
- [Versioning Scheme](#versioning-scheme)
- [Simplified Release Process](#simplified-release-process)

## Features, Commits, and Pull Requests

### Commit Messages
1. Keep titles short and describe the core change.
2. Use the commit message body for additional "what" and "why" details.

### Branching Strategy
1. Use `main` as the stable, production-ready branch and `develop` for integrating feature branches and testing.
2. Create branches from `develop` with the naming convention `feature/author/feature-name`. Valid prefixes are:
    - `feature/` for new features.
    - `bug/` for bug fixes.
    - `enhancement/` for enhancements like performance improvements, etc.
    - `refactor/` for refactoring.
    - `chore/` for maintenance tasks like updating dependencies, tests, etc.
3. Make regular commits with descriptive messages on the feature branch.
4. Create a pull request from the feature branch to `develop` for tracking changes and code review.
5. Merge the feature branch into `develop` after approval.
6. Regularly merge `develop` into `main` for stable production changes.
7. Delete feature branches after merging `develop` into `main`.
8. Keep your local `develop` branch updated with the remote.

### Hotfix Branches
1. Create a `hotfix` branch from `main` for critical bugs in production in the format `hotfix/author/issue-name`.
2. Make fixes in the `hotfix` branch and create a pull request to merge changes into `main` and `develop`.
3. After approval and passing tests, merge the `hotfix` branch into `main` and `develop`, then delete the branch.

### Merge Conflict Resolution
1. Review conflicting changes carefully.
2. Communicate with developers for clarity.
3. Select appropriate changes and ensure correctness.
4. Thoroughly test merged code.
5. Commit resolved merge and complete pull request.

### Pull Request Guidelines
1. Provide a clear, concise title.
2. Include a description of the branch's purpose.
3. Link to related issues or tickets so issues can be closed and users/clients can be notified.
4. Ensure all CI/CD checks pass.
5. Keep PRs small and focused for easier merging.

### Continuous Integration and Deployment
1. For new projects, set up a CI/CD pipeline that automatically builds, tests, and deploys the code from the `main` branch.
2. Configure the pipeline to run tests and security scans on pull requests targeting the `develop` branch to catch issues early.


## Release Cycle and Versioning

### Release Strategy

Lunabotics CDH repos will follow a flexible, feature-driven release strategy which allows for releases based on completed features or significant improvements, rather than adhering to a strict schedule.

- **Release Types:**
  1. Feature Releases: When significant new features or improvements are ready
  2. Hotfixes: As needed for critical issues

### Versioning Scheme

Lunabotics CDH repos will use Semantic Versioning (SemVer) to clearly communicate version changes. The version number format will be: MAJOR.MINOR.PATCH

- **MAJOR:** Incremented for significant changes or new features that may affect compatibility (breaking changes, major features, big refactors)
- **MINOR:** Incremented for new features or substantial improvements (non-breaking features)
- **PATCH:** Incremented for bug fixes and minor updates (hotfixes, chores, enhancements, refactors)

Examples:
- 1.0.0 - Initial release
- 1.1.0 - New feature added
- 1.1.1 - Bug fix
- 2.0.0 - Major update with potentially breaking changes

### Simplified Release Process

1. **Development**
   - Work on new features or improvements
   - Conduct basic testing as you go

2. **Pre-Release Check**
   - Review changes and ensure everything is working as expected
   - Update version number according to SemVer rules
   - Prepare a brief changelog
   - Run checks by Chief Engineer and Project Manager, other subteam leads

3. **Release**
   - Deploy the new version to the Lunabotics Robot or Simulation Software 
   - Communicate changes to the rest of the subteams and management (share the changelog)

4. **Post-Release**
   - Address any immediate feedback or issues
   - Begin work on the next set of features or improvements