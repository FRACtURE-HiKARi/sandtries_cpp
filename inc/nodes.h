/*
    The game objects.
*/

#pragma once

/*
    The base object of everything.
*/
class Node{

};

/*
    The base object of anything that is visible in the game.
*/

class Visible: public Node{
    virtual void show() = 0;
};

