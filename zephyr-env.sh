# Project-local paths loaded by this machine's .venv/bin/activate.
_cmu_project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ZEPHYR_BASE="$_cmu_project_root/.tools/zephyrproject/zephyr"
export ZEPHYR_SDK_INSTALL_DIR="$_cmu_project_root/.tools/zephyr-sdk-1.0.1"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export STM32_PRG_PATH="$_cmu_project_root/.tools/STM32CubeProgrammer/bin"
export BOARD=nucleo_f401re
export XDG_CACHE_HOME="$_cmu_project_root/.tools/cache"
export CCACHE_DIR="$_cmu_project_root/.tools/cache/ccache"
export CCACHE_TEMPDIR="$_cmu_project_root/.tools/cache/ccache/tmp"
PATH="$STM32_PRG_PATH:$ZEPHYR_SDK_INSTALL_DIR/hosttools/sysroots/x86_64-pokysdk-linux/usr/bin:$PATH"
export PATH
mkdir -p "$CCACHE_TEMPDIR"
unset _cmu_project_root
