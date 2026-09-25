# 💿 NVMe-for-Cact

<p align="center">
  <img src="https://img.shields.io/badge/version-2.0.0-green.svg?style=for-the-badge" alt="Version: 2.0.0">
  <img src="https://img.shields.io/badge/license-GPLv3-blue.svg?style=for-the-badge" alt="License: GPLv3">
  <img src="https://img.shields.io/badge/arch-i686-red.svg?style=for-the-badge" alt="Arch: i686">
  <img src="https://img.shields.io/badge/format-cctk-green.svg?style=for-the-badge" alt="Output: nvme.cctk">
  <img src="https://img.shields.io/badge/bus-PCI-blue.svg?style=for-the-badge" alt="PCI">
  <img src="https://img.shields.io/badge/irq-MSI--X_or_MSI-brightgreen.svg?style=for-the-badge" alt="MSI-X or MSI">
</p>

<p align="center">
  Out-of-tree <strong>NVMe</strong> driver → <strong><code>nvme.cctk</code></strong> for <strong>pci_load_module</strong>.<br>
  <strong>2.0.0:</strong> migrated from poll-only to <strong>MSI-X or MSI</strong> interrupt-driven I/O. Removed <code>pic_mask_line</code>/<code>pic_unmask_line</code>, added <code>msidev_register()</code>/<code>msidev_unregister()</code> with fallback to poll mode.
</p>

---

## 🔨 Building

**Recommended — full workspace**

```sh
ninja -C CactOS-x86_32/build-meson iso
```

**Standalone**

```sh
meson setup build-meson --cross-file cross/i686-cact-clang.ini
ninja -C build-meson          # → build-meson/nvme.cctk
ninja -C build-meson stage    # copy into ../LocalRepoCactOS-x86_32/lib/
ninja -C build-meson clean
```

Override paths if needed: `meson configure build-meson -Dkern_root=/custom/path -Dlocal_repo=/custom/path`.
