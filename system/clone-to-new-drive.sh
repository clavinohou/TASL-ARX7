#!/bin/bash
# Clone the running Ubuntu install (Kingston NVMe) onto the Fanxiang S660 NVMe,
# file by file, so it boots on its own with the Kingston removed.
# Run as root on the robot. ERASES the Fanxiang drive.
set -uo pipefail

DST_ID=/dev/disk/by-id/nvme-Fanxiang_S660_500GB_26030313611001145
NEW=/mnt/newroot
LOG_DIR=/home/arx/clone
FAILED="$LOG_DIR/rsync-errors.txt"

say() { printf '\n=== %s  %s\n' "$(date +%T)" "$*"; }
die() { printf '\n!!! ABORT: %s\n' "$*"; exit 1; }

[ "$(id -u)" -eq 0 ] || die "run as root"
mkdir -p "$LOG_DIR"

# ---- safety checks -------------------------------------------------------
DST=$(readlink -f "$DST_ID") || die "Fanxiang not found at $DST_ID"
[ -b "$DST" ] || die "$DST is not a block device"
ROOT_DISK=/dev/$(lsblk -no PKNAME "$(findmnt -no SOURCE /)")
EFI_DISK=/dev/$(lsblk -no PKNAME "$(findmnt -no SOURCE /boot/efi)")
[ "$DST" != "$ROOT_DISK" ] || die "$DST holds / — refusing"
[ "$DST" != "$EFI_DISK" ]  || die "$DST holds /boot/efi — refusing"
if lsblk -nro MOUNTPOINTS "$DST" | grep -q .; then die "$DST has mounted partitions"; fi
SIZE_GB=$(( $(blockdev --getsize64 "$DST") / 1000000000 ))
[ "$SIZE_GB" -ge 450 ] && [ "$SIZE_GB" -le 520 ] || die "unexpected size ${SIZE_GB} GB"
OLD_ROOT_UUID=$(findmnt -no UUID /)
OLD_EFI_UUID=$(findmnt -no UUID /boot/efi)
say "source: $ROOT_DISK (root $OLD_ROOT_UUID, efi $OLD_EFI_UUID)"
say "target: $DST ($DST_ID, ${SIZE_GB} GB) — erasing"

# ---- partition + format --------------------------------------------------
wipefs -a "$DST" || die "wipefs"
sgdisk --zap-all "$DST" >/dev/null
sgdisk -n1:0:+512M -t1:ef00 -c1:"EFI System Partition" \
       -n2:0:0     -t2:8304 -c2:"ubuntu-root" "$DST" || die "sgdisk"
partprobe "$DST"; udevadm settle; sleep 2
P1=${DST}p1; P2=${DST}p2
[ -b "$P1" ] && [ -b "$P2" ] || die "partitions did not appear"
mkfs.vfat -F32 -n EFI "$P1" || die "mkfs.vfat"
mkfs.ext4 -F -q -L ubuntu-root "$P2" || die "mkfs.ext4"
NEW_ROOT_UUID=$(blkid -s UUID -o value "$P2")
NEW_EFI_UUID=$(blkid -s UUID -o value "$P1")
say "new root $NEW_ROOT_UUID, new efi $NEW_EFI_UUID"

mkdir -p "$NEW"
mount "$P2" "$NEW" || die "mount root"
mkdir -p "$NEW/boot/efi"
mount "$P1" "$NEW/boot/efi" || die "mount efi"

# ---- copy ----------------------------------------------------------------
EXCL=(--exclude=/swapfile --exclude=/tmp/* --exclude=/var/tmp/* --exclude=/lost+found
      --exclude=/mnt/* --exclude=/media/*)
RS=(rsync -aAXHx --numeric-ids --info=progress2 --no-inc-recursive)

say "pass 1: copying / (35 GB)"
"${RS[@]}" "${EXCL[@]}" / "$NEW/" 2> >(tee "$FAILED" >&2)
RC1=$?
say "pass 1 rsync exit $RC1 (0 ok, 24 = files vanished mid-copy, 23 = some files unreadable)"
[ "$RC1" -eq 0 ] || [ "$RC1" -eq 23 ] || [ "$RC1" -eq 24 ] || die "rsync pass 1 failed ($RC1)"

say "pass 2: catching changes"
"${RS[@]}" --delete "${EXCL[@]}" / "$NEW/" 2> >(tee -a "$FAILED" >&2)
RC2=$?
say "pass 2 rsync exit $RC2"

say "copying old EFI partition"
rsync -rt /boot/efi/ "$NEW/boot/efi/" || die "efi copy"

# ---- fix up the copy -----------------------------------------------------
say "fstab -> new UUIDs"
sed -i "s/$OLD_ROOT_UUID/$NEW_ROOT_UUID/; s/$OLD_EFI_UUID/$NEW_EFI_UUID/" "$NEW/etc/fstab"
grep -v '^#' "$NEW/etc/fstab"

say "swapfile (2 GB)"
fallocate -l 2G "$NEW/swapfile" && chmod 600 "$NEW/swapfile" && mkswap "$NEW/swapfile" >/dev/null

say "installing GRUB on the Fanxiang (no firmware boot-entry changes)"
for d in dev dev/pts proc sys run; do mount --bind /$d "$NEW/$d"; done
mount --bind /sys/firmware/efi/efivars "$NEW/sys/firmware/efi/efivars" 2>/dev/null
chroot "$NEW" grub-install --target=x86_64-efi --efi-directory=/boot/efi \
       --bootloader-id=ubuntu --no-nvram || die "grub-install"
chroot "$NEW" grub-install --target=x86_64-efi --efi-directory=/boot/efi \
       --removable --no-nvram || die "grub-install --removable"
# signed GRUB reads a stub grub.cfg next to itself; make sure both copies point at the new root
for f in "$NEW/boot/efi/EFI/ubuntu/grub.cfg" "$NEW/boot/efi/EFI/BOOT/grub.cfg"; do
  [ -f "$f" ] || cp "$NEW/boot/efi/EFI/ubuntu/grub.cfg" "$f"
  sed -i "s/$OLD_ROOT_UUID/$NEW_ROOT_UUID/g" "$f"
done
chroot "$NEW" update-grub || die "update-grub"

# ---- checks --------------------------------------------------------------
say "checks"
echo "-- EFI files:"; find "$NEW/boot/efi/EFI" -type f | sed "s|$NEW/boot/efi||"
echo "-- stub configs:"; cat "$NEW/boot/efi/EFI/ubuntu/grub.cfg"
echo "-- grub.cfg root UUIDs:"; grep -o 'root=UUID=[0-9a-f-]*' "$NEW/boot/grub/grub.cfg" | sort | uniq -c
if grep -q "$OLD_ROOT_UUID" "$NEW/boot/grub/grub.cfg" "$NEW/etc/fstab" "$NEW/boot/efi/EFI/"*/grub.cfg; then
  echo "WARNING: old UUID still referenced:"; grep -l "$OLD_ROOT_UUID" "$NEW/boot/grub/grub.cfg" "$NEW/etc/fstab" "$NEW/boot/efi/EFI/"*/grub.cfg
fi
df -h "$NEW" "$NEW/boot/efi"
echo "-- unreadable files (rsync errors):"; grep -E 'failed|error|Input/output' "$FAILED" || echo "none"

say "unmounting"
umount "$NEW/sys/firmware/efi/efivars" 2>/dev/null
for d in run sys proc dev/pts dev; do umount "$NEW/$d"; done
umount "$NEW/boot/efi" "$NEW"
sync
say "DONE"
