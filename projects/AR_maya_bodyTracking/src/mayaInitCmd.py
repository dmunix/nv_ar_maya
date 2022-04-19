from maya import cmds

port = ":5055"
if port not in cmds.commandPort(q=1, lp=1):
    cmds.commandPort(name=port)

cmds.spaceLocator(n="locator1")


for i in range(34):
    cmds.joint(None, n="joint{}".format(i))
    