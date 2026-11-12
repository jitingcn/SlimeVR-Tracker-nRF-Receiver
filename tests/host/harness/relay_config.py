"""Select peer-relay configuration for source-extracted host fixtures."""


def select_relay(source, enabled=True):
    """Select the peer-relay branch before extracting C fixtures.

    Leave other configuration directives for the fixture's C compiler. Keep
    line counts stable so extraction diagnostics still refer to the source.
    """
    result = []
    stack = []
    active = True
    for line in source.splitlines(keepends=True):
        directive = line.strip()
        if directive.startswith(('#if ', '#ifdef ', '#ifndef ')):
            relay = directive in ('#if defined(CONFIG_RELAY)', '#if !defined(CONFIG_RELAY)')
            choice = enabled if directive == '#if defined(CONFIG_RELAY)' else not enabled
            stack.append((active, relay, choice))
            if relay:
                active = active and choice
                result.append('\n')
                continue
        elif directive.startswith(('#else', '#elif')) and stack:
            parent, relay, choice = stack[-1]
            if relay:
                active = parent and not choice
                result.append('\n')
                continue
        elif directive.startswith('#endif') and stack:
            parent, relay, _ = stack.pop()
            if relay:
                active = parent
                result.append('\n')
                continue
        result.append(line if active else '\n')
    return ''.join(result)
