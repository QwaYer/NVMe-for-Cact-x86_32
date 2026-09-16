# ==============================================================================
# NVMe module — Meson build system
# ==============================================================================
#
#   meson setup build-meson --cross-file cross/i686-cact-clang.ini \
#       -Dkern_root=../CactKernel-x86_32 -Dlocal_repo=../LocalRepoCactOS-x86_32
#   ninja -C build-meson          # build-meson/nvme.cctk
#   ninja -C build-meson stage    # copy into <local_repo>/lib
#
# The .cctk is a single relocatable object (ET_REL) that the kernel's PCI module
# loader maps and relocates, so it is compiled straight to that file instead of
# being archived.
# ==============================================================================

project('nvme-mod', 'c',
  meson_version: '>=1.0.0',
  default_options: [
    'c_std=gnu17',
    'warning_level=1',        # -Wall, as in the Makefile
    'buildtype=plain',
  ])

fs = import('fs')
bash = find_program('bash')
cc = meson.get_compiler('c')

src_root = meson.project_source_root()

# Path options are relative to this repo's root unless absolute.
_kern = get_option('kern_root')
kern_root = _kern.startswith('/') ? _kern : src_root / _kern
_local = get_option('local_repo')
local_repo = _local.startswith('/') ? _local : src_root / _local
install_dir = local_repo / 'lib'

# Kernel headers this module compiles against.
mod_includes = [
  kern_root / 'Cact/kernel/sync',
  kern_root / 'Cact/kernel/core',
  kern_root / 'Cact/drivers/pci/enum',
  kern_root / 'Cact/drivers/pci',
  kern_root / 'Cact/fs/vfs',
  kern_root / 'Cact/fs/vfs/devfs',
]

mod_c_args = [
  '-ffreestanding',
  '-fno-pie',
  '-fno-stack-protector',
  '-nostdlib',
  # Ring 0: the kernel has not enabled SSE for itself, so clang must not lower
  # struct copies to SSE2 (the same trap the kernel build hits).
  '-mno-mmx',
  '-mno-sse',
  '-mno-sse2',
  '-O2',
]
foreach d : mod_includes
  mod_c_args += '-I' + d
endforeach
mod_c_args += '-I' + src_root

nvme_cctk = custom_target('nvme.cctk',
  input: 'nvme_mod.c',
  output: 'nvme.cctk',
  command: cc.cmd_array() + ['-m32'] + mod_c_args +
           ['-MD', '-MF', '@DEPFILE@', '-c', '@INPUT@', '-o', '@OUTPUT@'],
  depfile: 'nvme.cctk.d',
  build_by_default: true)

# Meson reserves the name "install" for itself, hence "stage".
run_target('stage',
  command: [bash, '-c', 'set -e; mkdir -p "$1"; cp -f "$2" "$1/nvme.cctk"',
            'bash', install_dir, nvme_cctk])
