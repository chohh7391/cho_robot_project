#!/usr/bin/env python3

"""Generate a checkerboard calibration target as a PDF at exact millimetre size.

The board in ``targets/`` was produced by this script. It exists so the target is
reproducible and its dimensions live in source rather than only inside a binary
PDF -- the printed square size is an input to ``cameracalibrator --square``, and a
target nobody can regenerate is a number nobody can check.

The page is emitted at its true size (A4 = 297 x 210 mm), so printing at 100%
scale gives squares of exactly ``--square-mm``. Printers still lie; the README
says how to verify with callipers.
"""

import argparse

from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

# A4 landscape. Everything here is millimetres.
PAGE_WIDTH = 297.0
PAGE_HEIGHT = 210.0


def build(path, cols, rows, square, page_width, page_height):
    """Write a centred ``cols`` x ``rows`` checkerboard of ``square`` mm to *path*."""
    board_width = cols * square
    board_height = rows * square
    if board_width > page_width or board_height > page_height:
        raise SystemExit(
            f'{cols}x{rows} squares of {square} mm is {board_width:.0f}x'
            f'{board_height:.0f} mm and does not fit on {page_width:.0f}x'
            f'{page_height:.0f} mm.')

    offset_x = (page_width - board_width) / 2.0
    offset_y = (page_height - board_height) / 2.0

    # OpenCV needs a quiet zone around the outermost corners, or
    # findChessboardCorners starts losing the board at shallow angles. The paper
    # margin is only half the story: mounting the sheet on a larger white board
    # extends the quiet zone past the paper edge, which is the real fix on A4.
    # Half a square is the point below which the backing stops being optional.
    if min(offset_x, offset_y) < square / 2.0:
        print(f'warning: margin is {min(offset_x, offset_y):.1f} mm, under half a '
              f'{square:.0f} mm square; mount this on a white backing board.')

    page = canvas.Canvas(path, pagesize=(page_width * mm, page_height * mm))
    page.setTitle(f'Checkerboard {cols - 1}x{rows - 1} internal corners, '
                  f'{square} mm squares')
    page.setFillColorRGB(0, 0, 0)
    for row in range(rows):
        for col in range(cols):
            if (row + col) % 2 == 0:
                continue
            # PDF puts the origin bottom-left; rows are counted from the top.
            page.rect((offset_x + col * square) * mm,
                      (page_height - offset_y - (row + 1) * square) * mm,
                      square * mm, square * mm, stroke=0, fill=1)
    page.showPage()
    page.save()
    print(f'{path}: {cols}x{rows} squares ({cols - 1}x{rows - 1} internal corners), '
          f'{square} mm, board {board_width:.0f}x{board_height:.0f} mm, '
          f'margins {offset_x:.1f}/{offset_y:.1f} mm')


def main():
    """Parse arguments and emit the board."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--cols', type=int, default=10,
                        help='squares across; internal corners is one less')
    parser.add_argument('--rows', type=int, default=7,
                        help='squares down; internal corners is one less')
    parser.add_argument('--square-mm', type=float, default=25.0)
    parser.add_argument('--page-mm', type=float, nargs=2,
                        default=[PAGE_WIDTH, PAGE_HEIGHT],
                        metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('-o', '--output', default='checkerboard.pdf')
    args = parser.parse_args()

    # An odd/even pair of internal-corner counts is what removes the board's
    # 180-degree ambiguity; 9x6 is odd/even, 8x6 is not.
    if (args.cols - 1) % 2 == (args.rows - 1) % 2:
        print(f'warning: {args.cols - 1}x{args.rows - 1} internal corners have the '
              'same parity, so the board is symmetric under 180 degrees and its '
              'orientation is ambiguous.')

    build(args.output, args.cols, args.rows, args.square_mm,
          args.page_mm[0], args.page_mm[1])


if __name__ == '__main__':
    main()
