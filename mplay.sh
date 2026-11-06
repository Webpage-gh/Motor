#!/data/data/com.termux/files/usr/bin/sh
# mplay —— 把音频文件直接送到马达上播放（ffmpeg 解码 → motor stdin）
#
# 用法:
#   mplay <文件> [起始秒] [时长秒] [mag]
#   mplay --dry <文件> ...        只体检不振动（普通用户可跑）
set -u

DIR=$(cd "$(dirname "$0")" && pwd)
MOTOR="$DIR/motor"
DRY=0
FLAGS=""

usage() {
    sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
}

while [ $# -gt 0 ]; do
    case "$1" in
        --dry)  DRY=1; shift ;;
        -h|--help) usage ;;
        --*) echo "未知选项: $1" >&2; usage ;;
        *) break ;;
    esac
done

FILE=${1:-}
START=${2:-}
DUR=${3:-}
MAG=${4:-}

[ -n "$FILE" ] || usage
[ -f "$FILE" ] || { echo "找不到文件: $FILE" >&2; exit 1; }

# 切口 20ms 淡入/淡出：切段产生的陡边会被安全层当瞬态压掉（实测压到 56~68%）
AF="afade=t=in:st=0:d=0.02"
if [ -n "$DUR" ]; then
    OUTST=$(awk -v d="$DUR" 'BEGIN{printf "%.3f", (d > 0.04 ? d - 0.02 : 0)}')
    AF="$AF,afade=t=out:st=$OUTST:d=0.02"
fi

decode() {
    if [ -n "$START" ] && [ -n "$DUR" ]; then
        ffmpeg -v error -nostdin -ss "$START" -t "$DUR" -i "$FILE" -af "$AF" -f f32le -ac 1 -ar 24000 -
    elif [ -n "$START" ]; then
        ffmpeg -v error -nostdin -ss "$START" -i "$FILE" -af "$AF" -f f32le -ac 1 -ar 24000 -
    elif [ -n "$DUR" ]; then
        ffmpeg -v error -nostdin -t "$DUR" -i "$FILE" -f f32le -ac 1 -ar 24000 -
    else
        ffmpeg -v error -nostdin -i "$FILE" -f f32le -ac 1 -ar 24000 -
    fi
}

if [ "$DRY" = 1 ]; then
    decode | "$MOTOR" $FLAGS --dry stdin $MAG
else
    decode | su -c "$MOTOR $FLAGS stdin $MAG"
fi
