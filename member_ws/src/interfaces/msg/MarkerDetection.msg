uint16 id

# The marker's four corners, in the order they are printed on the panel:
# top-left, top-right, bottom-right, bottom-left of the marker itself, not of
# however it happens to sit in the image. A marker seen rotated still reports
# the same physical corner first, which is what lets a consumer pair these
# with the board-frame corners the challenge spec tabulates without deciding
# the correspondence again.
Pixel[4] corners
