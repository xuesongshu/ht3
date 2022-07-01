# How to use develop.cmd

1. Install FX3 SDK 1.3.4 with default options, do not change anything while wizard let you change

   You can skip this step if you already installed FX3 SDK 1.3.4.

2. Install Git 2.32.0 with default options, do not change anything while wizard let you change

   You can skip this step if you already installed Git 2.32.0.

3. Optional step. Install Notepad++ with default options, do not change anything while wizard let you change

   You can use other editor tool to edit makefile.

4. Prepare working directory. For example, `E:\in_use\FX3`.

   If you use other directory, please change `FX3FWROOT` variable in makefile synchrorously.

5. Copy following directories from FX3 SDK install directory to working directory
   * ARM GCC
   * bin
   * boot_lib
   * firmware
   * fw_build
   * fw_lib
   * util
6. Rename `ARM GCC` to `ARM_GCC`

   In order to avoid path space trouble.

7. Start `develop.cmd` that's provided by my project
8. Run `cs-make all`.

Now you can see this problem.
