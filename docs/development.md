# Development

## Building and flashing

```
pio run                      # build all targets
pio run -e <target>          # build specific target like: esp32,esp32s3,...
pio run -e esp32 -t upload   # build and flash device
pio test -e native           # run unit tests
pio check                    # run static analysis
pio run -t check_format      # check code style
pio run -e native -t format  # format code
```

## Docker

If you don't want to install PlatformIO, you can use the Docker environment.

General usage:
```
docker compose run --rm espfc <pio command>
```

Examples:

```
docker compose run --rm espfc pio run -e esp32
docker compose run --rm espfc pio test -e native
```

> [!NOTE]
> Keep in mind that flashing a device from Docker requires additional setup, because Docker is an isolated environment and host devices are not accessible by default.

## VSCode

This project is based on [PlatformIO](https://platformio.org/); it is recommended to install it as a VSCode extension.

1. If you don't have VSCode yet, visit https://code.visualstudio.com/download
2. Then install https://platformio.org/install/ide?install=vscode

## Contribution

If you want to include your changes in this project, you need to open a Pull Request (PR) on GitHub.

### How to contribute to the ESP-FC project

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
- Run tests and static analysis tools.
- Collect evidence that proves your changes do the job.

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

1. Provide a clear description of the change, explaining what problem you are trying to solve
2. Provide evidence that the feature you deliver works as expected and does not break any existing functionality
3. Address only one problem or feature in a PR
4. Ensure that unit tests are passing and static analysis tools do not report any errors
5. Avoid committing unnecessary changes
6. Untested changes will not be accepted
