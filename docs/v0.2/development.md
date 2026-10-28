# Development

## Building and flashing

```
pio run                      # build all targets
pio run -e <target>          # build specific target like: esp32,esp32s3,...
pio run -e esp32 -t upload   # build and flash device
pio test -e native           # run unit tests
pio check                    # run static anlysis
pio run -t check_format      # check code style
pio run -e native -t format  # format code
```

## Docker

If you don't want to install PlatformIO, you can use Docker Environment

general usage
```
docker compose run --rm espfc <pio command>
```

examples

```
docker compose run --rm espfc pio run -e esp32
docker compose run --rm espfc pio test -e native
```

> [!NOTE]
> Keep in mind, that to be able to flash device, additional effort is required as by default docker is isolated enviroment devices are't accessible.

## VSCode

This project is based on [platformio](https://platformio.org/), it is recommended to install it as VSCode IDE extension.

1. if you don't have VSCode yet? visit https://code.visualstudio.com/download
2. then install https://platformio.org/install/ide?install=vscode

## Contribution

If you want to include your changes in this project, you need to make PR (Pull Request) on github.

### How to contribute to Esp-fc project

1. Fork the repository

Click the Fork button on the project’s GitHub page to create your own copy.

2. Clone your fork

```
git clone https://github.com/your-username/project-name.git
```

3. Create a new branch

```
git checkout -b feature/my-change
```

4. Make your changes

- Edit files, add features, or fix bugs.
- Run tests and static anlysis tools.
- Collect eveidence that proves your changes do the job

5. Commit your changes

```
git add .
git commit -m "Add short description of changes"
```

6. Push the branch to your fork

```
git push origin feature/my-change
```

7. Open a Pull Request (PR)

Go to your fork on GitHub and click Compare & pull request. Add a clear title and description, then submit the PR. Wait for review. Project maintainers may request changes before merging your contribution.

### What requirements must be met for a PR to be accepted?

1. Provide clear change description with reasoning what type of problem you are trying to resolve
2. Provide evidence that feature you deliver works as expected and do not breaks any existing functionality
3. In PR address only one problem or feature
4. Ensure that unittests are passing and static alanysis tools do not report any errors
5. Avoid commiting unnecesary changes
6. Untested changes will not be accepted
