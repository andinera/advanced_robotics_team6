import maestro

if __name__ == '__main__':
        pololu = maestro.Controller()
        pololu.setTarget(0, 256)
