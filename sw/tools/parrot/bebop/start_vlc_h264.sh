#!/bin/sh

function start_stream
{
	killall -9 vlc
	vlc ftp://$ip/internal_000/stream.sdp --avcodec-hw=vaapi --no-xlib --no-interact --no-audio --clock-synchro 0 --no-sout-audio --network-caching $time_out --avcodec-skiploopfilter 4 --avcodec-codec h264 --sout-avcodec-hurry-up --avcodec-fast --udp-buffer 3000000 --live-caching $time_out --avcodec-skip-idct 4 --avcodec-skip-frame 2 --sout-schro-rate-control low_delay --screen-fps $fps --rtsp-frame-buffer-size 3000000 --h264-fps $fps --quiet
}

function usage
{
	echo "usage: start_h264_stream.sh -ac ID -t time_out -f FPS"
}

aircraft=1
time_out=350
fps=18

while [ "$1" != "" ]; do
    case $1 in
        -ac | --aircraft )      shift
                                aircraft=$1
                                ;;
        -t | --time_out )    	shift
        			time_out=$1
                                ;;
        -f | --fps )    	shift
        			fps=$1
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
ip=192.168.0.$aircraft
fi

start_stream
