#!/bin/sh
function rand(){
    min=$1
    max=$(($2-$min+1))
    num=$(($RANDOM+1000000000))
    echo $(($num%$max+$min))
}

echo "start to do-test.sh"
i=1
f=$(ls /dev/mmcblk*p*)
if [ -z "$f" ];then
        f=$(ls /dev/mmcblk*)
fi
if [ -n "$f" ];then
        while true
        do
                folder=$(printf "/mnt/sdcard/%d/" "$i")
                if [ ! -d "$folder" ];then
                        break
                fi
                i=$(($i+1))
        done
        i=$(($i-1))
        folder=$(printf "/mnt/sdcard/%d/" "$i")
        echo "find cur folder $i $folder"

        log_dir="$folder/log"
        echo "log dir $log_dir"
        mkdir $log_dir

        cp /tmp/rtt-log.txt $log_dir/rtt-fastae.log
        cp /tmp/pts*.txt $log_dir/
        cp /tmp/rkaiq*.log $log_dir/
        dmesg &> $log_dir/kernel.log
        make_meta -d /dev/block/by-name/meta > $log_dir/meta.txt
        (cat /dev/mpi/vlog; cat /dev/mpi/valloc; cat /dev/mpi/vmcu; cat /dev/mpi/vrgn; cat /dev/mpi/vsys; echo ==========================================; cat /dev/mpi/vlog; cat /dev/mpi/valloc; cat /dev/mpi/vmcu; cat /dev/mpi/vrgn; cat /dev/mpi/vsys;) |tee > $log_dir/dev-mpi.log
        cat /proc/rkisp-vir0 &> $log_dir/proc-rkisp-vir0-1.log
        sleep .2
        cat /proc/rkisp-vir0 &> $log_dir/proc-rkisp-vir0-2.log
        cat /proc/rkisp-vir1 &> $log_dir/proc-rkisp-vir1-1.log
        sleep .2
        cat /proc/rkisp-vir1 &> $log_dir/proc-rkisp-vir1-2.log
        cat /proc/rkcif-mipi-lvds &> $log_dir/proc-rkcif-mipi-lvds-1.log
        sleep .2
        cat /proc/rkcif-mipi-lvds &> $log_dir/proc-rkcif-mipi-lvds-2.log
        cat /proc/vcodec/enc/venc_info &> $log_dir/proc-vcodec-enc-venc_info-1.log
        sleep .2
        cat /proc/vcodec/enc/venc_info &> $log_dir/proc-vcodec-enc-venc_info-2.log
        sync
        umount /mnt/sdcard/
fi

killall ao_record_demo
killall ao_record_service
sleep 2

frame=$(rand 40 139)
echo "pre-recording will exit after $frame frame"
export lppr_wakeup=2
export lppr_wakeup_cnt=$frame
/usr/bin/tinyplay /oem/usr/share/enter_pre_recording_2ch.wav -D 0 -d 0
/usr/bin/lppr
