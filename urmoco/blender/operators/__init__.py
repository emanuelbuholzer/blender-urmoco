from urmoco.scheduler import Scheduler
from urmoco.config import Config


def get_operators(config: Config, scheduler: Scheduler):
    operators = []

    from .freedrive import get_operators

    operators += get_operators(config, scheduler)

    from .power_off import get_operators

    operators += get_operators(config, scheduler)

    from .capturing import get_operators

    operators += get_operators(config, scheduler)

    from .stop import get_operators

    operators += get_operators(config, scheduler)

    from .sync import get_operators

    operators += get_operators(config, scheduler)

    from .transfer import get_operators

    operators += get_operators(config, scheduler)

    from .unlock import get_operators

    operators += get_operators(config, scheduler)

    from .startup import get_operators

    operators += get_operators(config, scheduler)

    return operators
