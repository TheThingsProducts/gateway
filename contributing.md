# Contributing Guidelines

Thank you for considering contributing to this project! Please follow the guidelines below to ensure a smooth contribution process.

## Getting Started
1. **Fork the Repository**: Click the "Fork" button on the repository’s page.
2. **Clone Your Fork** with its submodules:
   ```sh
   git clone --recurse-submodules https://github.com/<your-account>/gateway.git
   ```
3. **Set up the toolchain**: the firmware builds with Microchip XC32 v1.42 and MPLAB X v3.45 only; see
   [`firmware/README.md`](firmware/README.md). Build with `firmware/compile.sh`.
4. **Create a Branch** from `upd-only`, the branch that holds the current development. Use a descriptive
   branch name for your changes:
   ```sh
   git checkout -b feature/your-feature-name upd-only
   ```

## Making Changes
- Follow the coding style of the file you change. Sources under `firmware/src/` use CRLF line endings;
  keep them.
- Keep commits focused and meaningful. Write clear commit messages and name the issue a commit belongs
  to (`#12`).
- A firmware change adds its section to [`changelog.md`](changelog.md). A release also bumps
  `firmware/version/` and gets its release notes in `releases/<version>/README.md`.
- Do not commit built firmware (`firmware.hex`, `checksums`): images are published as assets of the
  GitHub release.
- Pages in `webui/static/` are compiled into the firmware. After changing them, regenerate the image with
  `python3 webui/gen_mpfs.py` and commit `firmware/src/mpfs_img2.c` with the change; see
  [`webui/README.md`](webui/README.md).
- Things that have caused faults before:
  - `double` is 32 bits wide in XC32 v1.42. Do not use floating point where the digits matter (for
    example a frequency in Hz); use integer arithmetic.
  - RAM is almost completely in use. After a build, check that the startup stack is still there:
    `xc32-nm -n <elf> | grep -E ' (_heap|_splim|_stack)$'`.
- Keep the layout of the original repository: the branches `develop`, `master`, `beta` and `stable` and
  the workflow in `.github/workflows/` belong to the update channel design of The Things Products and
  are not renamed or removed.

## Submitting a Pull Request
1. **Push to Your Fork**:
   ```sh
   git push origin feature/your-feature-name
   ```
2. **Open a Pull Request**:
   - Navigate to the original repository.
   - Click on "New Pull Request".
   - Select `upd-only` as the base branch, select your branch and provide a clear description of your
     changes: what was wrong or missing, what the change does and how it was tested on a gateway.

## Code Review Process
- PRs will be reviewed by maintainers.
- Be open to feedback and make necessary changes.
- Keep your branch up to date with `upd-only`.

## Reporting Issues
- Check if the issue has already been reported in the [issues](https://github.com/pe1mew/gateway/issues).
- Provide detailed information: the firmware version (Info page of the gateway or the boot banner on the
  debug console), how the gateway is connected (Ethernet, WiFi), and steps to reproduce the issue. A
  capture of the debug console (USART4, 115200 8N1) helps most.
- Use clear and concise language.

## Community Standards
- Follow the [Code of Conduct](code_of_conduct.md).
- Be respectful and collaborative.

Happy Coding! 🚀
