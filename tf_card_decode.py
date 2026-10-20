#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TF 卡告警快照解码器
====================
读取 SD 卡(或 dd/Win32DiskImager 导出的 .img 镜像)的裸扇区, 解析 snap_store.v
写入的告警快照:

  每次告警占 3 个扇区(从扇区 8 开始, 环形 96 扇区 = 最多 32 次告警):
    扇区 N   : 512B 二进制日志(魔数/帧号/明火面积/烟头计数/包围盒/类型标志)
    扇区 N+1 : 缩略图前 512B  (40x22 灰度, 共 880 字节有效)
    扇区 N+2 : 缩略图后 512B  (剩余 144 字节为 0xFF 填充)

输出:
  1. 每张缩略图放大为 PGM 灰度图 (文件名 thumb_XX.pgm, 可用 IrfanView/GIMP 打开,
     或用 ImageMagick:  magick thumb_XX.pgm thumb_XX.png)
  2. 控制台打印日志表 (帧号/面积/包围盒/类型)

用法:
  python tf_card_decode.py --image dump.img            # 读镜像文件(推荐, 安全)
  python tf_card_decode.py --drive 3                   # 直接读 \\.\PhysicalDrive3 (需管理员)
  python tf_card_decode.py --image dump.img --scale 32 # 放大倍数(默认16)
  python tf_card_decode.py --image dump.img --outdir out # 输出目录

说明: 缩略图是 40x22 (原图每 32x32 取 1 像素), 原始分辨率很低, 放大后必然模糊,
     这是设计取舍(省扇区/省时间), 不是解码错误。
"""
import struct, sys, os, argparse

LOG_BASE   = 8      # 日志起始扇区
RING_SECTORS = 96   # 环形扇区数
THUMB_W    = 40     # 缩略图宽 (1280/32)
THUMB_H    = 22     # 缩略图高 (720/32)
SECTOR     = 512
MAGIC      = b'\xa5\xa5\xa5\xa5'


def parse_log(b):
    """解析 512B 日志扇区, 魔数不匹配返回 None"""
    if b[:4] != MAGIC:
        return None
    frame     = struct.unpack_from('<I', b, 4)[0]
    fire_area = b[8] | (b[9] << 8) | ((b[10] & 0x0F) << 16)
    cig_count = b[12] | (b[13] << 8) | ((b[14] & 0x0F) << 16)
    fx0 = b[16] | ((b[17] & 0x07) << 8)
    fx1 = b[18] | ((b[19] & 0x07) << 8)
    fy0 = b[20] | ((b[21] & 0x03) << 8)
    fy1 = b[22] | ((b[23] & 0x03) << 8)
    cx0 = b[24] | ((b[25] & 0x07) << 8)
    cx1 = b[26] | ((b[27] & 0x07) << 8)
    cy0 = b[28] | ((b[29] & 0x03) << 8)
    cy1 = b[30] | ((b[31] & 0x03) << 8)
    fv  = b[32] & 1
    cv  = (b[32] >> 1) & 1
    ts  = struct.unpack_from('<I', b, 34)[0]
    return dict(frame=frame, fire_area=fire_area, cig_count=cig_count,
                fx0=fx0, fx1=fx1, fy0=fy0, fy1=fy1,
                cx0=cx0, cx1=cx1, cy0=cy0, cy1=cy1,
                fire_v=fv, cig_v=cv, ts=ts)


def upscale_nearest(gray, scale):
    """40x22 灰度最近邻放大 scale 倍 -> 字节串"""
    w, h = THUMB_W * scale, THUMB_H * scale
    out = bytearray(w * h)
    for y in range(h):
        sy = y // scale
        row = gray[sy * THUMB_W:(sy + 1) * THUMB_W]
        for x in range(w):
            out[y * w + x] = row[x // scale]
    return w, h, bytes(out)


def write_pgm(path, w, h, data):
    with open(path, 'wb') as f:
        f.write(('P5\n%d %d\n255\n' % (w, h)).encode())
        f.write(data)


def write_png(path, w, h, data):
    try:
        from PIL import Image
        Image.frombytes('L', (w, h), data).save(path)
        return True
    except ImportError:
        return False


def read_image(args):
    if args.image:
        with open(args.image, 'rb') as f:
            return f.read()
    else:
        # 直接读物理盘 (Windows 需管理员权限)
        drive = '\\\\.\\PhysicalDrive%d' % args.drive
        print('读取 %s ... (若提示权限错误, 请用管理员运行, 或先用 dd/HxD 导出镜像)' % drive)
        with open(drive, 'rb') as f:
            # 只读前 200 扇区足够 (8+96=104 扇区)
            return f.read(200 * SECTOR)


def main():
    ap = argparse.ArgumentParser(description='TF卡告警快照解码器')
    ap.add_argument('--image', help='SD卡镜像文件(.img/.bin)')
    ap.add_argument('--drive', type=int, help='直接读 PhysicalDriveN (需管理员)')
    ap.add_argument('--scale', type=int, default=16, help='缩略图放大倍数(默认16)')
    ap.add_argument('--outdir', default='tf_out', help='输出目录(默认 tf_out)')
    args = ap.parse_args()

    if not args.image and args.drive is None:
        ap.error('必须指定 --image 或 --drive')

    img = read_image(args)
    total_sectors = len(img) // SECTOR
    print('读取 %d 字节 = %d 扇区' % (len(img), total_sectors))
    if total_sectors < LOG_BASE + RING_SECTORS:
        print('警告: 镜像不足 %d 扇区, 可能不是完整 SD 卡转储' % (LOG_BASE + RING_SECTORS))

    os.makedirs(args.outdir, exist_ok=True)
    n_found = 0

    for i in range(RING_SECTORS // 3):
        sec = LOG_BASE + i * 3
        if (sec + 2) * SECTOR > len(img):
            break
        log = parse_log(img[sec * SECTOR:(sec + 1) * SECTOR])
        if log is None:
            continue  # 该槽位空

        gray = img[(sec + 1) * SECTOR:(sec + 3) * SECTOR][:THUMB_W * THUMB_H]
        w, h, up = upscale_nearest(gray, args.scale)
        base = os.path.join(args.outdir, 'thumb_%02d' % i)
        png_ok = write_png(base + '.png', w, h, up)
        if not png_ok:
            write_pgm(base + '.pgm', w, h, up)

        typ = ('明火' if log['fire_v'] else '') + ('+' if log['fire_v'] and log['cig_v'] else '') + ('烟头' if log['cig_v'] else '')
        if not typ:
            typ = '无'
        print('[槽位%02d] 扇区%d 帧号=%d 类型=%s 明火面积=%d 烟头计数=%d 明火框=(%d,%d)-(%d,%d) 烟头框=(%d,%d)-(%d,%d)' % (
            i, sec, log['frame'], typ, log['fire_area'], log['cig_count'],
            log['fx0'], log['fy0'], log['fx1'], log['fy1'],
            log['cx0'], log['cy0'], log['cx1'], log['cy1']))
        n_found += 1

    print('\n共解析 %d 条有效告警, 缩略图已存到 %s/' % (n_found, args.outdir))
    if n_found == 0:
        print('未找到有效数据。可能原因: ①还没触发过告警 ②读错了盘(用 --drive 看盘号) ③扇区被覆盖')


if __name__ == '__main__':
    main()
