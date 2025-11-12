# Detects AprilTags, outputs rotation

import sensor, image, time, math

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # we run out of memory if the resolution is much bigger...
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
t0 = time.time_ns()

# Note! Unlike find_qrcodes the find_apriltags method does not need lens correction on the image to work.

# The apriltag code supports up to 6 tag families which can be processed at the same time.
# Returned tag objects will have their tag family and id within the tag family.

tag_families = 0
tag_families |= image.TAG36H11 # comment out to disable this family (default family)

# What's the difference between tag families? Well, for example, the TAG16H5 family is effectively
# a 4x4 square tag. So, this means it can be seen at a longer distance than a TAG36H11 tag which
# is a 6x6 square tag. However, the lower H value (H5 versus H11) means that the false positve
# rate for the 4x4 tag is much, much, much, higher than the 6x6 tag. So, unless you have a
# reason to use the other tags families just use TAG36H11 which is the default family.


while(True):
    img = sensor.snapshot()
    img.lens_corr(1.8) # strength of 1.8 is good for the 2.8mm lens.
    t = (time.time_ns() - t0)/1e9
    try:
        apriltags = img.find_apriltags(families=tag_families)
    except MemoryError as e:
        apriltags = []
    print_args = []
    for tag in apriltags:
        img.draw_rectangle(tag.rect(), color = (255, 0, 0))
        img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))

        print_args.append((tag.id(), (180 * tag.rotation() / math.pi), tag.cx(), tag.cy()))
    print_args.append(t)
    try:
        print(print_args)
    except:
        pass
