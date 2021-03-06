import scipy.io
import MatToGaziboWorld
import sys


if __name__ == "__main__":
    if len(sys.argv) != 4 and len(sys.argv) != 5:
        print(
            "Takes 3 arguments: <mat-file-path> <init-pos-x,init-pos-y> <world-name> (<scaling-divider>)"
        )
        exit(1)
    if len(sys.argv) == 5:
        try:
            scaling = int(sys.argv[4])
        except:
            print("Scaling must be a number")
            exit(1)
    else:
        scaling = 1

    worldName = sys.argv[3]
    pos = sys.argv[2].split(",")
    try:
        initX = int(pos[0])
        initY = int(pos[1])
    except:
        print("Position must by in format: int,int")
        exit(1)

    sdfHead = MatToGaziboWorld.createSDF()
    world = MatToGaziboWorld.createWorld(sdfHead)
    myModel = MatToGaziboWorld.newModel(world, "myModel")
    # create the file structure
    try:
        mat = scipy.io.loadmat(sys.argv[1])
    except:
        print("Error in opening file")
        exit(1)

    rects = MatToGaziboWorld.matToRect(mat)

    for num, rect in enumerate(rects):
        MatToGaziboWorld.createWall(
            rect.x_min,
            rect.y_min,
            rect.x_max,
            rect.y_max,
            myModel,
            f"Wall_{num}",
            initX,
            initY,
            scaling,
        )

    MatToGaziboWorld.createGaziboFiles(sdfHead, worldName)