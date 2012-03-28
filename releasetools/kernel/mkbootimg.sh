#!/sbin/sh
echo \#!/sbin/sh > /tmp/createnewboot.sh

###Add zcache to the kernels cmdline.
#cmdline=$(cat /tmp/boot.img-cmdline)
#searchString="zcache"
#case $cmdline in
#  "$searchString" | "$searchString "* | *" $searchString" | *" $searchString "*) ;;  
#  *)
#	echo $(cat /tmp/boot.img-cmdline)\ zcache > /tmp/boot.img-cmdline ;;
#esac

echo /tmp/mkbootimg --kernel /tmp/zImage --ramdisk /tmp/boot.img-ramdisk.gz --cmdline \"$(cat /tmp/boot.img-cmdline)\" --base $(cat /tmp/boot.img-base) --output /tmp/newboot.img >> /tmp/createnewboot.sh
chmod 777 /tmp/createnewboot.sh
/tmp/createnewboot.sh
return $?
