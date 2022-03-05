XDIM = 1000; YDIM = 600

class Env:
    def __init__(self):
        self.x_range = (0, XDIM)
        self.y_range = (0, YDIM)
        # self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    # @staticmethod
    # def obs_rectangle():
    #     obs_rectangle = [[120, 100, 100, 200],
    #                     [350, 350, 200, 100],
    #                     [350, 600, 50, 200],
    #                     [750, 600, 50, 200],
    #                     [800, 200, 100, 200]]
    #     return obs_rectangle

    # @staticmethod
    # def obs_circle():
    #     obs_cir = [ [120, 500, 50],
    #                 [400, 100, 50],
    #                 [500, 600, 50],
    #                 [600, 200, 50],
    #                 [700, 450, 50]]

    #     return obs_cir
    # # obs_sydney
    @staticmethod
    def obs_circle():
        obs_cir = [[730, 740, 80],
                    [400, 300, 80],
                    #[300/10, 500/10, 60/10],
                    #[640/10, 480/10, 50/10],
                    [170, 140, 90],
                    #[600,100,70],    
                    [600,100,70],#scen2
                    [940,510,80]]
        return obs_cir
    @staticmethod
    def obs_rectangle():
        obs_rectangle = [[740, 140, 170, 140],
                        [320, 570, 200, 220],
                        [580,420,120,150], #scen2
                        [120,410,170,120]#scen2
                        ]
        return obs_rectangle
    
    # obs_wood
    # @staticmethod
    # def obs_circle():
    #     obs_cir = [#[400, 300, 80],#scen1
    #                 [500,310,70],#scen2
    #                 [850, 500, 80],
    #                 [620, 440, 50],
    #                 #[170, 140, 90], #scen1
    #                 [200,260,90],#scen2
    #                 [600,130,90]#scen2
    #                 ]
    #     return obs_cir
    # @staticmethod
    # def obs_rectangle():
    #     obs_rectangle = [[740, 140, 170, 140],
    #                     [260,30,150,120],#scen2
    #                     [80,400,190,130]#scen2
    #                     #[130, 400, 150, 120]#scen1
    #                     ]
    #     return obs_rectangle
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, YDIM],
            [0, YDIM, XDIM, 1],
            [1, 0, XDIM, 1],
            [XDIM, 1, 1, YDIM]
        ]
        return obs_boundary

    # @staticmethod
    # def obs_rectangle():
    #     obs_rectangle = [[500, 250, 300, 20],
    #                     [0, 150, 300, 20],
    #                     [180, 380, 300, 20],
    #                     [650, 50, 20, 100],
    #                     [650, 350, 20, 200],
    #                     [300, 150, 20, 120],
    #                     [180, 500, 120, 20],
    #                     [400, 500, 20, 100]]
    #     return obs_rectangle

    # @staticmethod
    # def obs_circle():
    #     obs_cir = [ [120, 240, 40],
    #                 [460, 160, 40],
    #                 [470, 520, 40]]

    #     return obs_cir
