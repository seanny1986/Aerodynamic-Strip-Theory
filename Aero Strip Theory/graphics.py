import pygame
import environments

class Graphics():
    def __init__(self, environment, width, height):
        self.environment = environment
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('Wireframe Display')
        self.background = (10,10,50)

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
            self.screen.fill(self.background)
            pygame.display.flip()


    