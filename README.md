# FRC Team 5165 Common code repository
This repository contains code that is shared between applications and robots that are created by FRC Team 5165.

## Usage
Add this library by using git command `git submodule add https://github.com/team5308/common.git common` in the project's root directory. This will clone this repo into the directory `common`.

### Using the Common library

A dependency on the common library can be created by adding the following to
your `build.gradle` file:
```gradle
dependencies {
    ...
    compile project(':common')
    ...
}
```
The following also needs to be added to your `settings.gradle` file:
```gradle
include ':common'
```

# Reference
This repo is enlightened by FRC 2910 Common Repo.
