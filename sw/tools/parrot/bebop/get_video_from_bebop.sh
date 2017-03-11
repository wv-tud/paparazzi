#!/bin/sh

function get_video
{
	ffmpeg -analyzeduration 100M -probesize 10M -c h264 -enable_er -r $rate -an -sn -i ftp://$ip/internal_000/$file_name.h264 -vcodec libx264 -ss 1 -map_chapters -1 -map_metadata -1 -an -sn "~/Videos/$file_name-$aircraft.mp4"
	echo "Do you want to remove the video? [Y/N]:"
	read rmfile
	if [ "$rmfile" = "Y" ]; then
		{ echo "rm /data/ftp/internal_000/$file_name.h264"; sleep 5; } | telnet $ip
		echo "Removed"
	else
		echo "Not removed"
	fi
}

function usage
{
	echo "usage: start_h264_stream.sh -ac ID -f file_name"
}

aircraft=1
file_name="ar_filter_demo"
rate=18

while [ "$1" != "" ]; do
    case $1 in
        -ac | --aircraft )      shift
                                aircraft=$1
                                ;;
        -f | --file )    	shift
        			file_name=$1
                                ;;
        -r | --rate )    	shift
        			rate=$1
                                ;;
        -h | --help )           usage
                                exit
                                ;;
        * )                     usage
                                exit 1
    esac
    shift
done

if [ "$aircraft" = "1" ]; then
ip=192.168.42.1
else
ip=192.168.40.$aircraft
fi

get_video
