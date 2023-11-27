# all floating points represent percentage of image resolution height or width
# this class is necessary, because we want to use slightly different parameters for
# grouping objects, depending on whether we are looking at a wide or narrow image

class CameraSpecificDetection:
    WIDE = 'WIDE'
    STANDARD = 'STANDARD'

    Settings = {
        WIDE: {
            'VERTICAL_LINES': {
                # How much (percent) horizontal difference between emitter centers is allowed, to consider the emitters part of same line
                'HORIZONTAL_LEEWAY':0.03
            },
            'SQUARES': {
                'VERTICAL_LINE_CENTER_LEEWAY':0.08, # percent (pixelwise) centers of lines can be vertically different, to still be considered associated
                'MAX_LINE_HEIGHT_DIFF':0.2,
                'MAX_HORIZONTAL_DIST':0.5,
                'MAX_WIDTH_HEIGHT_RATIO':1.5 # check for squarage, vs rectanglage
            },
            'TRIANGLES': {
                'VERTICAL_LEEWAY':0.05, # percent (pixelwise) centers of lines can be vertically different, to still be considered associated
                'MAX_HORIZONTAL_DIST':0.5,
                'MIN_WIDTH': 0.04,
                'MAX_WIDTH_HEIGHT_RATIO':1.5 # check for triangle shape, vs hugely stretched. should be almost as wide as high
            }
        },
        STANDARD: {
            'VERTICAL_LINES': {
                # How much (percent) horizontal difference between emitter centers is allowed, to consider the emitters part of same line
                'HORIZONTAL_LEEWAY':0.05
            },
            'SQUARES': {
                'VERTICAL_LINE_CENTER_LEEWAY':0.08, # percent (pixelwise) centers of lines can be vertically different, to still be considered associated
                'MAX_LINE_HEIGHT_DIFF':0.2, # was 0.1, before adding squarage check
                'MAX_HORIZONTAL_DIST':0.5, # was 0.2, before adding squarage check (below)
                'MAX_WIDTH_HEIGHT_RATIO':1.5 # check for squarage, vs rectanglage
            },
            'TRIANGLES': {
                'VERTICAL_LEEWAY':0.05, # percent (pixelwise) centers of lines can be vertically different, to still be considered associated
                'MAX_HORIZONTAL_DIST':0.5, # was .2 before adding ratio check
                'MIN_WIDTH': 0.04,
                'MAX_WIDTH_HEIGHT_RATIO':1.5 # check for triangle shape, vs hugely stretched. should be almost as wide as high
            }
        }

    }