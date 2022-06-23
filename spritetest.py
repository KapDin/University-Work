import pygame, sys

pygame.init()
clock = pygame.time.Clock()

class Vehicle(pygame.sprite.Sprite):
    def __init__(self, config={}):
        super().__init__()
        self.default_init()
        for attr, val in config.items():
            setattr(self, attr, val)
        
    def default_init(self):
        self.width = 30
        self.height = 30
        self.pos_x = 100
        self.pos_y = 200
        self.color = (122, 211, 156)

screen_width = 1920
screen_height = 1080
screen = pygame.display.set_mode((screen_width, screen_height))
zoom = 5
offset = (0, 0)
mouse_down = False

vehicle = Vehicle()
vehicle_group = pygame.sprite.Group()
vehicle_group.add(vehicle)



while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
            
    pygame.display.flip()
    vehicle_group.draw(screen)
    clock.tick(60)