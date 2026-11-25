#!/bin/bash
# Finish the clone after the copy: swapfile, GRUB, checks, power off.
# Run as root on the robot after clone-resume.sh copied the files; powers off only if every check passes.
set -uo pipefail

DST_ID=/dev/disk/by-id/nvme-Fanxiang_S660_500GB_26030313611001145
NEW=/mnt/newroot
LOG_DIR=/home/arx/clone
FAILED="$LOG_DIR/rsync-errors-resume.txt"


say() { printf '\n=== %s  %s\n' "$(date +%T)" "$*"; }
die() { printf '\n!!! ABORT: %s\n' "$*"; exit 1; }
[ "$(id -u)" -eq 0 ] || die "run as root"


DST=$(readlink -f "$DST_ID")
P1=${DST}p1; P2=${DST}p2
ROOT_DISK=/dev/$(lsblk -no PKNAME "$(findmnt -no SOURCE /)")
[ "$DST" != "$ROOT_DISK" ] || die "$DST holds / — refusing"
findmnt -n "$NEW" >/dev/null || mount "$P2" "$NEW" || die "mount root"
findmnt -n "$NEW/boot/efi" >/dev/null || mount "$P1" "$NEW/boot/efi" || die "mount efi"
[ "$(findmnt -no SOURCE "$NEW")" = "$P2" ] || die "$NEW is not $P2"
OLD_ROOT_UUID=$(findmnt -no UUID /)
OLD_EFI_UUID=$(findmnt -no UUID /boot/efi)
NEW_ROOT_UUID=$(blkid -s UUID -o value "$P2")
NEW_EFI_UUID=$(blkid -s UUID -o value "$P1")
say "resuming into $P2 ($NEW_ROOT_UUID)"

say "swapfile (2 GB)"
rm -f "$NEW/swapfile"
fallocate -l 2G "$NEW/swapfile" && chmod 600 "$NEW/swapfile" && mkswap "$NEW/swapfile" >/dev/null || die "swapfile"

say "installing GRUB on the Fanxiang (no firmware boot-entry changes)"
for d in dev dev/pts proc sys run; do mount --bind /$d "$NEW/$d"; done
mount --bind /sys/firmware/efi/efivars "$NEW/sys/firmware/efi/efivars" 2>/dev/null
chroot "$NEW" grub-install --target=x86_64-efi --efi-directory=/boot/efi \
       --bootloader-id=ubuntu --no-nvram || die "grub-install"
chroot "$NEW" grub-install --target=x86_64-efi --efi-directory=/boot/efi \
       --removable --no-nvram || die "grub-install --removable"
for f in "$NEW/boot/efi/EFI/ubuntu/grub.cfg" "$NEW/boot/efi/EFI/BOOT/grub.cfg"; do
  [ -f "$f" ] || cp "$NEW/boot/efi/EFI/ubuntu/grub.cfg" "$f"
  sed -i "s/$OLD_ROOT_UUID/$NEW_ROOT_UUID/g" "$f"
done
chroot "$NEW" update-grub || die "update-grub"
umount "$NEW/sys/firmware/efi/efivars" 2>/dev/null
for d in run sys proc dev/pts dev; do umount "$NEW/$d"; done

# ---- checks --------------------------------------------------------------
say "checks"
OK=1
check() { if eval "$2"; then echo "  ok   $1"; else echo "  FAIL $1"; OK=0; fi; }
check "fallback bootloader EFI/BOOT/BOOTX64.EFI" "[ -s $NEW/boot/efi/EFI/BOOT/BOOTX64.EFI ]"
check "EFI/ubuntu/grubx64.efi"                  "[ -s $NEW/boot/efi/EFI/ubuntu/grubx64.efi ]"
check "stub grub.cfg points at new root"        "grep -q $NEW_ROOT_UUID $NEW/boot/efi/EFI/ubuntu/grub.cfg"
check "fallback stub points at new root"        "grep -q $NEW_ROOT_UUID $NEW/boot/efi/EFI/BOOT/grub.cfg"
check "grub.cfg boots new root"                 "grep -q root=UUID=$NEW_ROOT_UUID $NEW/boot/grub/grub.cfg"
check "no old root UUID left"                   "! grep -qs $OLD_ROOT_UUID $NEW/boot/grub/grub.cfg $NEW/etc/fstab $NEW/boot/efi/EFI/*/grub.cfg"
check "fstab has new root + efi"                "grep -q $NEW_ROOT_UUID $NEW/etc/fstab && grep -q $NEW_EFI_UUID $NEW/etc/fstab"
check "kernel + initrd present"                 "ls $NEW/boot/vmlinuz-* $NEW/boot/initrd.img-* >/dev/null"
check "kernel modules for $(uname -r)"          "[ -d $NEW/lib/modules/$(uname -r) ]"
check "nvidia module present"                   "find $NEW/lib/modules/$(uname -r) -name 'nvidia.ko*' | grep -q ."
check "ssh host keys + authorized_keys"         "ls $NEW/etc/ssh/ssh_host_*_key >/dev/null && [ -s $NEW/home/arx/.ssh/authorized_keys ]"
check "NetworkManager connections"              "ls $NEW/etc/NetworkManager/system-connections/ | grep -q ."
check "same file count (within 1%)"             "a=\$(df --output=iused / | tail -1); b=\$(df --output=iused $NEW | tail -1); [ \$((a - b)) -lt \$((a / 100)) ]"
echo "-- used:"; df -h --output=source,used,target / "$NEW"
echo "-- unreadable files this run:"; grep -o 'mapping "[^"]*"' "$FAILED" | sort -u || echo "none"

umount "$NEW/boot/efi" "$NEW"
sync
if [ "$OK" -eq 1 ]; then
  say "ALL CHECKS PASSED — powering off in 15 s"
  sleep 15
  systemctl poweroff
else
  say "CHECKS FAILED — not powering off"
fi
