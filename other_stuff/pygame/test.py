import pygame as pg
import os

_image_library = {}
def get_image(path):
    global _image_library
    image = _image_library.get(path)
    if image == None:
        canonicalized_path = path.replace('/',os.sep).replace('\\',os.sep)
        image = pg.image.load(canonicalized_path)
        _image_library[path] = image
    return image

pg.init()
screen = pg.display.set_mode((400,300))
done = False

pg.draw.rect(screen,(0,128,255),pg.Rect(30,30,60,60))

is_blue = True
x = 30
y = 30

clock = pg.time.Clock()

while not done:

    for event in pg.event.get():
        if event.type ==pg.KEYDOWN and event.key == pg.K_SPACE:
            is_blue = not is_blue
 
        if event.type == pg.QUIT:
            done = True

    pressed = pg.key.get_pressed()
    if pressed[pg.K_w]: y -= 1
    if pressed[pg.K_s]: y += 1
    if pressed[pg.K_a]: x -= 1
    if pressed[pg.K_d]: x += 1

    screen.fill((255,255,255))
    screen.blit(get_image('tsen_hsv.png'),(10,10))

    if is_blue: color = (0,128,255)
    else: color = (255,100,0)
    pg.draw.rect(screen,color,pg.Rect(x,y,60,60))   




    pg.display.flip()
    clock.tick(60)
