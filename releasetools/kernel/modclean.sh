#!/sbin/sh
busybox rm /system/lib/modules/*
busybox rm /system/etc/init.d/90zram
busybox mv /system/bin/mpdecision /system/bin/mpdecision_dis
busybox mv /system/bin/thermald /system/bin/thermald_dis
return $?
