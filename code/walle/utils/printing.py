

def ros_print(node, *msgs, sep=' '):
    ret = ''
    for msg in msgs:
        ret += str(msg) + str(sep)

    node.get_logger().info(ret)
