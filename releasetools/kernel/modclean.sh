#!/sbin/sh
busybox rm /system/lib/modules/*
busybox rm /system/etc/init.d/90zram
return $?
