import pygame

pygame.init()
clock = pygame.time.Clock()
font = pygame.font.SysFont('', 30)

timeevent = pygame.USEREVENT
pygame.time.set_timer(timeevent, 1000)
counter = 1
check = True
screen = pygame.display.set_mode((1000, 1000))

while check:

    clock.tick(144)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            check = False
        elif event.type == timeevent:
            counter +=1
    
    screen.fill((255, 255, 255))
    screen.blit(font.render(f'time={clock}', False, (0, 0, 0)), (0, 0))
    screen.blit(font.render(f'count={counter}', False, (0, 0, 0)), (200, 200))

    pygame.display.flip()