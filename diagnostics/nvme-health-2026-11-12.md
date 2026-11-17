# ARX robot NVMe health checkout

Collected on November 12, 2026 at 5:44 p.m. EST (22:44:47 UTC) over SSH. The root filesystem is on `/dev/nvme0n1p2`. The device identified itself as a Kingston `SNV3S1000G` with firmware `P3BR0A23`. Linux reported a device size of 931.5 GiB.

| Measurement | Observed value |
| --- | --- |
| Media and data integrity errors | **5,311**, up from 5,305 at the prior read |
| Critical warning flags | `0` |
| Command error log entries | `0`. All four returned slots were empty. |
| Power-on hours | 68 |
| Power cycles | 229 |
| Unsafe shutdowns | 124 |
| Available spare and threshold | 85% available, 10% threshold |
| Estimated life used | 0% reported |
| Composite temperature | 50 °C (323 K) |
| Temperature sensors | 86 °C, 60 °C, 50 °C (rounded) |

Linux logged NVMe status type `0x2`, code `0x81` on repeated reads, including at LBAs `546507008`, `546510360`, and `1662390248`. These status values identify unrecovered reads. The media-error counter records unrecovered data-integrity errors independently of the warning flags and command error log. It can count retries to the same address. The results support treating the SSD as failing. The exact hardware failure mode remains undetermined. [NVMe status code definitions](https://nvmexpress.org/wp-content/uploads/NVM-Express-1_4b-2020.09.21-Ratified.pdf) · [NVMe health log field definitions](https://nvmexpress.org/wp-content/uploads/NVM-Express-Base-Specification-Revision-2.3-2025.08.01-Ratified.pdf)

The installed SciPy LAPACK file returned an I/O error during the earlier teleop check. A clean SciPy wheel loaded successfully in RAM. During this checkout, Linux reported over 90% short-term full I/O pressure while the filesystem journal thread and `dpkg` waited on storage. SSH remained responsive. GNOME Remote Desktop accepted connections and timed out during display setup. The GNOME compositor still timed out after writes recovered. It was waiting on `drm_modeset_lock`, and the boot log contained warnings from the Nouveau graphics driver. The cause of the display lock remains undetermined.

The checkout used read-only NVMe administrative queries and existing kernel logs. The raw capture includes device information, SMART and error-log JSON, kernel errors, and the I/O snapshot. Preserve any remaining important data and replace or evaluate the SSD before relying on the robot for operation. [Raw checkout](nvme-checkout-2026-11-12.txt)

## Dependency reinstall

Disk writes recovered while Ubuntu's update waited at a `libpam-cap` configuration prompt. NumPy 2.2.6, SciPy 1.15.3, and Pygame 2.6.1 were reinstalled from checksum-verified wheels into `/home/arx/Documents/Teleop Test Enviroment/.deps` on the ext4 root filesystem. A fresh Python process imported all three packages and both ARX extensions from the test folder. SciPy's `Rotation` import passed and the updated launcher ran base and arm simulations. The system Python installation outside the test folder still has the original packages. [NumPy wheel](https://pypi.org/project/numpy/2.2.6/) · [SciPy wheel](https://pypi.org/project/scipy/1.15.3/) · [Pygame wheel](https://pypi.org/project/pygame/2.6.1/)

## Firmware follow-up

`nvme fw-log` reported active slot 1 with `P3BR0A23`. The NVMe controller reported unrecovered reads. The logs establish a storage reliability fault and leave the underlying firmware, controller, or media cause open. The 68 power-on hours make ordinary wear an unlikely explanation. Kingston lists applicable updates and release notes in Kingston SSD Manager for Windows. No public Kingston advisory confirming a fix for this exact revision was found. Check the drive directly attached to a Windows machine or ask Kingston support to evaluate the model, firmware, and checkout after backing up any remaining important data. [Kingston NV3 support](https://www.kingston.com/en/support/technical/products/snv3s) · [Kingston firmware guidance](https://www.kingston.com/en/support/technical/ksm-firmware-update)
