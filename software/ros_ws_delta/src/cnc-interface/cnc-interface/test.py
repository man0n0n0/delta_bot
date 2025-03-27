def parse_command(command_string):
    command = ""
    args = []
    kargs = {}

    if command_string[:12] == "executeGcode":
        gcodelist = command_string[13:]
        print(gcodelist)
        gcodelist = ast.literal_eval(gcodelist)
        command = "executeGcode"
        args = gcodelist
        print(type(args))

    elif command_string[:14] == "moveTipToPoint":
        pass

    else:
        raise ValueError("no command found")

    # self.get_logger().info(f'parse result: command {command}')
    return command, args, kargs

import ast
print(parse_command("executeGcode ['G0 Z20 F1500;', 'G28', 'G90']"))