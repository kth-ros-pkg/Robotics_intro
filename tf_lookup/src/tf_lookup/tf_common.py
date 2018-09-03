
def sanitize_frame(frame):
    return frame.lstrip("/")

def key_from_transform(target, source):
    return "{}@{}".format(target, source)
