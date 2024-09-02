if __name__=='__main__':
    '''
        Assume the start position is always the origin
        Subscribe to the topic from teleop-twist-keyboard
        With every velocity update, integrate it to find the position of the drone 
        Publish the current position of the drone us tf
    ''' 