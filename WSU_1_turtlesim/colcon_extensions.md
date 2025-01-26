---
sort: 1
---

# `python3-colcon-common-extensions`

The **`python3-colcon-common-extensions`** package is a set of extensions for **Colcon**, a command-line tool used to build and manage **ROS 2** workspaces and other multi-package projects.

## Purpose

Colcon is the recommended build tool for ROS 2. While the core **Colcon** tool provides basic build functionality, the **`python3-colcon-common-extensions`** package includes a collection of commonly used plugins and extensions that enhance Colcon's functionality for ROS 2 projects. 

These extensions simplify tasks like dependency management, workspace overlaying, and package-specific builds.

## Features of `python3-colcon-common-extensions`

1. **Enhanced Build and Test Options**:
   - Adds commonly needed build tools and testing plugins.
   - Supports building specific subsets of packages or skipping certain packages.

2. **Dependency Handling**:
   - Improves dependency resolution for multi-package builds.
   - Automatically detects and processes package dependencies.

3. **Customizable Build Workflows**:
   - Adds more options for customizing builds, such as skipping packages, parallel builds, or selective clean-up.

4. **Workspace Management**:
   - Streamlines overlaying workspaces (using `colcon build` to extend an existing workspace).
   - Provides tools to handle installation directories and build artifacts.

5. **Ease of Use**:
   - Improves the overall Colcon experience by adding helpful commands and features that are commonly needed in ROS 2 development workflows.

## Installing `python3-colcon-common-extensions`

To install this package on a Debian-based system (like Ubuntu):

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

Add this to .bashrc

```bash
# Enable Colcon argument autocompletion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

Source .bashrc

```bash
source ~/.bashrc
```

## Benefits
- Reduces the manual configuration of build settings.
- Automates common tasks, making development faster and more intuitive.
- Provides a more robust toolset for managing complex ROS 2 workspaces.

This package is essential for most ROS 2 developers, as it provides the functionality needed for efficient and scalable workspace and package management.


