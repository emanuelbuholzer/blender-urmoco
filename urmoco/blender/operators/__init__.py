def get_operators(config, urmoco_in_queue, urmoco_out_queue):
    operators = []

    from .freedrive import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .power_off import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .power_on import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .shooting import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .stop import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .sync import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .transfer import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    from .unlock import get_operators
    operators += get_operators(config, urmoco_in_queue, urmoco_out_queue)

    return operators
