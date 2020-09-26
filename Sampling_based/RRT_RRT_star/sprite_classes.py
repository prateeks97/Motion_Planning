import pygame

BLACK = (0,0,0)
GRAY = (100, 100, 100)
NAVYBLUE = ( 60, 60, 100)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = ( 0, 255, 0)
BLUE = ( 0, 0, 255)
YELLOW = (255, 255, 0)
ORANGE = (255, 128, 0)
PURPLE = (255, 0, 255)
CYAN = ( 0, 255, 255)
GREY = (220,220,220)


class car_orientation(pygame.sprite.Sprite):
    def __init__(self,world_scale,center, corners, color):
        super().__init__()
        cornerx = []
        cornery = []
        lst = []
        # pygame.sprite.Sprite.__init__(self)
        #pygame.draw.circle(screen,YELLOW,(int(world_scale*(start_vert[0] - world_xmin)), int(world_scale*(-start_vert[1] - world_ymin))),mp)
        self.image = pygame.Surface((world_scale*10, world_scale*10), pygame.SRCALPHA)
        # pygame.draw.polygon(self.image,color,corners)
        # pygame.gfxdraw.filled_circle(self.image, int(world_scale*2.5), int(world_scale*2.5), int(world_scale*1), color)
        self.rect = self.image.get_rect()
        self.rect.center = (world_scale*(center[0]-world_xmin), world_scale*(-center[1] - world_ymin))
        adjust = np.asarray(self.rect.topleft)
        print(adjust)
        for l in range(0,len(corners)):
            cornerx.append(corners[l][0] - adjust[0])
            cornery.append(corners[l][1] - adjust[1])
            lst.append(tuple(np.asarray([cornerx[l],cornery[l]])))
        # print("tuple" ,[tuple(np.asarray(cornerx[0],cornery[0])),tuple(np.asarray(cornerx[0],cornery[0])),tuple(np.asarray(cornerx[0],cornery[0]))])
        pygame.draw.polygon(self.image,color,lst)

class vertex_sprite(pygame.sprite.Sprite):
    def __init__(self,world_scale,center,radius):
        super().__init__()
        cornerx = []
        cornery = []
        lst = []
        # pygame.sprite.Sprite.__init__(self)
        #pygame.draw.circle(screen,YELLOW,(int(world_scale*(start_vert[0] - world_xmin)), int(world_scale*(-start_vert[1] - world_ymin))),mp)
        self.image = pygame.Surface((world_scale*2*radius, world_scale*2*radius), pygame.SRCALPHA)
        # pygame.draw.polygon(self.image,color,corners)
        pygame.gfxdraw.filled_circle(self.image, int(world_scale*radius), int(world_scale*radius), int(world_scale*radius), BLACK)
        self.rect = self.image.get_rect()
        self.rect.center = (world_scale*(center[0]-world_xmin), world_scale*(-center[1] - world_ymin))
