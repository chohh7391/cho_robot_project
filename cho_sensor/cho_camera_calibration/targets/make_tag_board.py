"""An A4 tag36h11 calibration board, PRE-SCALED for a printer that shrinks.

This bench's printer fits the A4 PAGE to its printable area, which is a fixed
91.92% -- not a function of how much ink is on the page, so drawing smaller
does not avoid it. Two prints measured it:

    70 mm drawn -> 64.35 measured    0.9193
    64 mm drawn -> 59    measured    0.9219 (a rounded reading of 58.83)
    273 / 297                        0.9192   <- the printable height ratio

So every dimension here is authored in PRINTED millimetres and multiplied by
1/0.91919 on the way to the page. Print it the SAME way as before and the
ruler reads 100.0 mm and the black squares are 70.0 mm.

THAT MAKES THIS FILE A TRAP FOR ANYONE WHO PRINTS IT AT 100%: they get 76 mm
tags. The sheet says so on its face, and the ruler is the check -- the printed
artefact is the thing that is 70 mm, never the PDF.

At 70 mm printed a 2x3 block is 262.5 mm tall, which leaves no footer, so the
ruler and the legend live in the side margins. That is also why the board is
2x3 and not a roomier 2x2: board rotation is constrained by the SPREAD of the
tags -- the thing a single planar marker measures worst -- and 87.5 x 175 mm
of centre span beats 87.5 x 87.5 by more than four tags' worth.

Vector cells, not a bitmap: a printer resampling a raster tag softens exactly
the edges the quad detector needs. The bits come from libapriltag through
render_tag, checked against the two tag images already in cho_description_fr5.
"""
import ctypes
import os
import sys

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
import numpy as np


"""(embedded) Read tag36h11 bitmaps from the DETECTOR'S OWN family definition.

The codes, the bit positions and the border width come out of libapriltag via
ctypes rather than out of a table typed in here, because the one expensive
failure mode for a printed tag is an image that looks right and decodes to
nothing. Verified below against the two rendered tags already in the repo.
"""


class Family(ctypes.Structure):
    _fields_ = [
        ('ncodes', ctypes.c_uint32),
        ('codes', ctypes.POINTER(ctypes.c_uint64)),
        ('width_at_border', ctypes.c_int),
        ('total_width', ctypes.c_int),
        ('reversed_border', ctypes.c_bool),
        ('nbits', ctypes.c_uint32),
        ('bit_x', ctypes.POINTER(ctypes.c_uint32)),
        ('bit_y', ctypes.POINTER(ctypes.c_uint32)),
        ('h', ctypes.c_uint32),
        ('name', ctypes.c_char_p),
        ('impl', ctypes.c_void_p),
    ]


_lib = ctypes.CDLL(os.environ.get('APRILTAG_LIB', 'libapriltag.so.3'))
_lib.tag36h11_create.restype = ctypes.POINTER(Family)
_FAMILY = _lib.tag36h11_create().contents


def info():
    return dict(name=_FAMILY.name.decode(), ncodes=_FAMILY.ncodes,
                nbits=_FAMILY.nbits, width_at_border=_FAMILY.width_at_border,
                total_width=_FAMILY.total_width,
                reversed_border=bool(_FAMILY.reversed_border))


def bitmap(tag_id):
    """Build a total_width x total_width array, 0 = black, 1 = white."""
    fam = _FAMILY
    if not 0 <= tag_id < fam.ncodes:
        raise ValueError(f'tag36h11 has {fam.ncodes} codes; {tag_id} is not one')
    size = fam.total_width
    border = (size - fam.width_at_border) // 2

    # White page, then the black border ring the family declares, then the bits.
    img = np.ones((size, size), dtype=np.uint8)
    img[border:border + fam.width_at_border, border:border + fam.width_at_border] = 0
    if fam.reversed_border:
        img[border:border + fam.width_at_border,
            border:border + fam.width_at_border] = 1
        inner = border + 1
        img[inner:size - inner, inner:size - inner] = 0

    code = fam.codes[tag_id]
    for bit in range(fam.nbits):
        # Bit nbits-1 is the most significant, as decode_quad walks them.
        value = (code >> (fam.nbits - 1 - bit)) & 1
        img[border + fam.bit_y[bit], border + fam.bit_x[bit]] = value
    return img


#: What the driver does to the page. Measured, twice, and equal to the ratio of
#: the printable height to A4's. Re-measure if the printer changes.
DRIVER_SCALE = 273.0 / 297.0
PRE = 1.0 / DRIVER_SCALE

# ----------------------------------------------- everything below is PRINTED mm
TAG_MM = 70.0                      # the BLACK SQUARE, which is what tag_size means
CELLS = info()['total_width']      # 10
BLACK = info()['width_at_border']  # 8
CELL_MM = TAG_MM / BLACK           # 8.75
PLATE_MM = CELL_MM * CELLS         # 87.5
MARK_MM = 2.0
RULER_MM = 100.0

BOARD_IDS = [[10, 11], [12, 13], [14, 15]]     # rows, top to bottom
VESSEL_IDS = [(0, 'beaker'), (1, 'flask')]

PAGE_W, PAGE_H = 210.0, 297.0      # drawn mm; the driver shrinks this whole page


def d(printed_mm):
    """Convert printed millimetres to the millimetres to draw."""
    return printed_mm * PRE


def draw_tag(pdf, tag_id, x, y):
    """Draw one tag, plate lower-left at DRAWN (x, y); black cells only."""
    grid = bitmap(tag_id)
    cell = d(CELL_MM)
    for row in range(CELLS):
        for col in range(CELLS):
            if grid[row][col]:
                continue                       # white: leave the paper
            # Row 0 of the bitmap is the TOP of the tag; PDF y grows upward.
            pdf.rect((x + col * cell) * mm, (y + (CELLS - 1 - row) * cell) * mm,
                     cell * mm, cell * mm, stroke=0, fill=1)


def corner_marks(pdf, x, y, w, h):
    arm = d(MARK_MM)
    pdf.setLineWidth(0.3)
    for cx, cy in ((x, y), (x + w, y), (x, y + h), (x + w, y + h)):
        pdf.line((cx - arm) * mm, cy * mm, (cx + arm) * mm, cy * mm)
        pdf.line(cx * mm, (cy - arm) * mm, cx * mm, (cy + arm) * mm)


def vertical_ruler(pdf, x, y):
    """Draw a ruler up the page, reading RULER_MM once printed."""
    length, tick = d(RULER_MM), d(1.8)
    pdf.setLineWidth(0.3)
    pdf.line(x * mm, y * mm, x * mm, (y + length) * mm)
    for step in range(0, int(RULER_MM) + 1, 10):
        here = y + d(float(step))
        long_tick = tick if step % 50 == 0 else tick * 0.6
        pdf.line(x * mm, here * mm, (x + long_tick) * mm, here * mm)


def rotated(pdf, x, y, size, body, limit):
    """Draw text bottom-to-top at DRAWN (x, y), refusing an overrun."""
    font = 'Helvetica'
    width = pdf.stringWidth(body, font, size) / mm
    if width > limit:
        raise SystemExit(f'legend is {width:.1f} mm against {limit:.1f}: {body[:50]}...')
    pdf.saveState()
    pdf.translate(x * mm, y * mm)
    pdf.rotate(90)
    pdf.setFont(font, size)
    pdf.drawString(0, 0, body)
    pdf.restoreState()


def board_page(pdf):
    cols, rows = len(BOARD_IDS[0]), len(BOARD_IDS)
    width, height = d(cols * PLATE_MM), d(rows * PLATE_MM)
    x0, y0 = (PAGE_W - width) / 2.0, (PAGE_H - height) / 2.0
    arm = d(MARK_MM)
    if x0 - arm < 0 or y0 - arm < 0:
        raise SystemExit(f'board is {width:.1f} x {height:.1f} mm drawn; '
                         f'it does not fit a {PAGE_W:.0f} x {PAGE_H:.0f} page')

    for row_index, row in enumerate(BOARD_IDS):
        for col_index, tag_id in enumerate(row):
            draw_tag(pdf, tag_id, x0 + col_index * d(PLATE_MM),
                     y0 + (rows - 1 - row_index) * d(PLATE_MM))
    corner_marks(pdf, x0, y0, width, height)

    # No footer fits under a 262.5 mm block, so the legend runs up the margins:
    # its label first, then the ruler, so neither reaches under the tags.
    rotated(pdf, 3.0, y0, 6.0,
            f'{RULER_MM:.0f} mm AFTER PRINTING - measure this first', height)
    vertical_ruler(pdf, 5.5, y0)

    rotated(pdf, 203.2, y0, 6.0,
            f'PRE-SCALED for a printer that shrinks A4 to '
            f'{100 * DRIVER_SCALE:.2f}%. Print it the SAME way, NOT at 100%.', height)
    rotated(pdf, 206.2, y0, 6.0,
            f'tag36h11 ids {BOARD_IDS[0][0]}-{BOARD_IDS[-1][-1]} row-major from '
            f'top-left. printed mm: square {TAG_MM:.1f}, plate {PLATE_MM:.1f}, '
            f'cell {CELL_MM:.2f}, pitch {PLATE_MM:.1f}, corner marks '
            f'{cols * PLATE_MM:.1f} x {rows * PLATE_MM:.1f}. '
            f'set tag_size from the MEASURED square.', height)
    pdf.showPage()


def vessel_page(pdf):
    """Lay out two vessel tags stacked, with room for the same ruler.

    At the pre-scaled plate size two of these are 190 mm of a 297 mm page, so
    the gaps are what is left over rather than what looks nice -- computed up
    front, because deriving the ruler's place from the loop variable put it off
    the bottom of the page the first time.
    """
    plate, arm = d(PLATE_MM), d(MARK_MM)
    gap = d(8.0)
    top = PAGE_H - d(10.0) - plate
    tops = [top - index * (plate + gap) for index in range(len(VESSEL_IDS))]
    x0 = d(14.0)
    lowest = tops[-1]
    if lowest - arm < d(4.0):
        raise SystemExit('vessel tags do not fit the page at this size')

    for (tag_id, label), y in zip(VESSEL_IDS, tops):
        draw_tag(pdf, tag_id, x0, y)
        corner_marks(pdf, x0, y, plate, plate)
        pdf.setFont('Helvetica', 8)
        pdf.drawString((x0 + plate + d(7.0)) * mm, (y + plate / 2.0) * mm,
                       f'id {tag_id} -> {label}  ({TAG_MM:.1f} mm printed)')

    pdf.setFont('Helvetica-Bold', 7.0)
    pdf.drawString((x0 + plate + d(7.0)) * mm, (tops[0] + plate - d(6.0)) * mm,
                   f'vessel tags, PRE-SCALED for {100 * DRIVER_SCALE:.2f}%.')
    pdf.setFont('Helvetica', 6.5)
    pdf.drawString((x0 + plate + d(7.0)) * mm, (tops[0] + plate - d(10.0)) * mm,
                   'print the SAME way as the board, not at 100%. '
                   'cut on the corner marks.')
    rotated(pdf, 3.0, lowest, 6.0,
            f'{RULER_MM:.0f} mm AFTER PRINTING', d(RULER_MM) + d(20.0))
    vertical_ruler(pdf, 5.5, lowest)
    pdf.showPage()


out = sys.argv[1] if len(sys.argv) > 1 else 'tag_board_70mm_prescaled.pdf'
pdf = canvas.Canvas(out, pagesize=A4)
pdf.setTitle(f'tag36h11 {TAG_MM:.0f} mm board, pre-scaled {100 * DRIVER_SCALE:.2f}%')
board_page(pdf)
vessel_page(pdf)
pdf.save()
print(f'wrote {out}')
print(f'  drawn at x{PRE:.5f} so the print lands at {TAG_MM:.1f} mm')
print(f'  printed: tag {TAG_MM}, plate {PLATE_MM}, pitch {PLATE_MM}, '
      f'centre span {PLATE_MM} x {2 * PLATE_MM} mm')
print(f'  drawn:   tag {d(TAG_MM):.3f}, plate {d(PLATE_MM):.3f} mm')
