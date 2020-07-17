////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stlastar.h" // See header for copyright and usage information
#include "Observer.h"
#include "Subject.h"

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <list>
#include <vector>


#include <SFML/Graphics.hpp>


#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

using namespace std;

// Global data

enum class GameEvent{
    search, quit, left, right, up, down, noop
};

int gamecharacter = 2;

// The world map

const int MAP_WIDTH = 20;
const int MAP_HEIGHT = 20;

int world_map[ MAP_WIDTH * MAP_HEIGHT ] =
        {

                // 0001020304050607080910111213141516171819
                1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 00
                1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,   // 01
                1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 02
                1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 03
                1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 04
                1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 05
                1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 06
                1,9,9,9,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,   // 07
                1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 08
                1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,   // 09
                1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,   // 10
                1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,   // 11
                1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 12
                1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 13
                1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 14
                1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 15
                1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 16
                1,1,9,9,9,9,9,9,9,1,1,1,9,9,9,1,9,9,9,9,   // 17
                1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 18
                1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 19

        };

// map helper functions

int GetMap(int x , int y)
{
    if (x<0 || x>MAP_WIDTH || y<0 || y>MAP_HEIGHT)
    {
        return 9;
    }
    return world_map[y* MAP_WIDTH +x];
}
// Definitions

class MapSearchNode:public Subject
{
public:
    int x;	 // the (x,y) positions of the node
    int y;

    MapSearchNode()
    {
        x = y = 0;

    }
    MapSearchNode( int px, int py )
    {
        x=px;
        y=py;
    }

    //Astar functions
    float GoalDistanceEstimate( MapSearchNode &nodeGoal );
    bool IsGoal( MapSearchNode &nodeGoal );
    bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
    float GetCost( MapSearchNode &successor );
    bool IsSameState( MapSearchNode &rhs );

    void move(int _x,int _y);

    void PrintNodeInfo();

    //Override functions
    void subscribe(Observer *o) override;

    void unsubscribe(Observer *o) override;

    void notify() override;

private:
    std::list<Observer*> observers;
};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

    // same state in a maze search is simply when (x,y) are the same
    return (x == rhs.x) &&
           (y == rhs.y);

}

void MapSearchNode::PrintNodeInfo()
{
    char str[100];
    sprintf( str, "My hero position : (%d,%d)\n", this->x,this->y);
    cout << str;
    notify();
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
    return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

    return (x == nodeGoal.x) &&
           (y == nodeGoal.y);

}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

    int parent_x = -1;
    int parent_y = -1;

    if( parent_node )
    {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }


    MapSearchNode NewNode;

    // push each possible move except allowing the search to go backwards

    if( (GetMap( x-1, y ) < 9)
        && !((parent_x == x-1) && (parent_y == y))
            )
    {
        NewNode = MapSearchNode( x-1, y );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x, y-1 ) < 9)
        && !((parent_x == x) && (parent_y == y-1))
            )
    {
        NewNode = MapSearchNode( x, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x+1, y ) < 9)
        && !((parent_x == x+1) && (parent_y == y))
            )
    {
        NewNode = MapSearchNode( x+1, y );
        astarsearch->AddSuccessor( NewNode );
    }


    if( (GetMap( x, y+1 ) < 9)
        && !((parent_x == x) && (parent_y == y+1))
            )
    {
        NewNode = MapSearchNode( x, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }

    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
    return static_cast<float>( GetMap( x, y ));
}

void MapSearchNode::subscribe(Observer *o)
{
    observers.push_back(o);

}

void MapSearchNode::unsubscribe(Observer *o)
{
    observers.remove(o);

}

void MapSearchNode::notify()
{
    for (auto &observer : observers)
        observer->update();

}

void MapSearchNode::move(int _x, int _y)
{
    this->x = _x;
    this->y = _y;
}


class PosPlayer :public Observer{
public:
    explicit PosPlayer (MapSearchNode* MSN)
    {
        this->subject = MSN;
        attach();
    }

    ~PosPlayer() override
    {
        detach();
    }

    void attach() override
    {
        subject->subscribe(this);

    }

    void detach() override
    {
        subject->unsubscribe(this);

    }

    void update() override
    {
        this->x = subject->x;
        this->y = subject->y;
        show();
    }

    void show()
    {


        world_map[this->y * MAP_WIDTH + this->x] = gamecharacter;
        for(int i=0 ; i<MAP_WIDTH*MAP_HEIGHT ; i++)
        {
            std::cout<<world_map[i];
            if( (i + 1)% MAP_WIDTH == 0)
            {
                std::cout<< "\n";
            } else
            {
                std::cout<<" ";
            }
        }
        std::cout<<"GameCharacter position: -X: "<< this->x <<" -Y: "<< this->y <<std::endl;
    }

private:
    int x;
    int y;
    MapSearchNode* subject;
};

//Game Class
class GameApp{
public:
    GameApp()
    {
        initGameCharacterNode();
        initPosPlayer();
    }

    bool updateGame(const GameEvent &gameEvent)
    {
        switch (gameEvent)
        {
            case GameEvent::quit: {
                return true;
            }
            case GameEvent::up:{

            }
            case GameEvent::left:{

            }
            case GameEvent::down:{

            }
            case GameEvent::right:{

            }
            case GameEvent::search:{
                unsigned int SearchCount = 0;
                const unsigned int NumSearches = 1;
                while(SearchCount < NumSearches)
                {

                    //Create a start state
                    heroStart->x = 10;
                    heroStart->y = 10;

                    world_map[heroStart->y * MAP_WIDTH + heroStart->x] =  gamecharacter;


                    // Define the goal state

                    heroEnd->x = 12;
                    heroEnd->y = 8;


                    // Set Start and goal states

                    aStarSearch.SetStartAndGoalStates( *heroStart, *heroEnd);

                    unsigned int SearchState;
                    unsigned int SearchSteps = 0;

                    do
                    {
                        SearchState = aStarSearch.SearchStep();

                        SearchSteps++;

#if DEBUG_LISTS

                        cout << "Steps:" << SearchSteps << "\n";

			int len = 0;

			cout << "Open:\n";
			MapSearchNode *p = astarsearch.GetOpenListStart();
			while( p )
			{
				len++;
	#if !DEBUG_LIST_LENGTHS_ONLY
				((MapSearchNode *)p)->PrintNodeInfo();
	#endif
				p = astarsearch.GetOpenListNext();

			}

			cout << "Open list has " << len << " nodes\n";

			len = 0;

			cout << "Closed:\n";
			p = astarsearch.GetClosedListStart();
			while( p )
			{
				len++;
	#if !DEBUG_LIST_LENGTHS_ONLY
				p->PrintNodeInfo();
	#endif
				p = astarsearch.GetClosedListNext();
			}

			cout << "Closed list has " << len << " nodes\n";
#endif

                    }
                    while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

                    if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
                    {
                        cout << "Search found goal state\n";

                        MapSearchNode* node = aStarSearch.GetSolutionStart();

#if DISPLAY_SOLUTION
                        cout << "Displaying solution\n";
#endif
                        int steps = 0;

                        node->PrintNodeInfo();
                        for( ;; )
                        {

                            node = aStarSearch.GetSolutionNext();


                            if( !node )
                            {
                                break;
                            }

                            hero_position = new PosPlayer (node);

                            node->PrintNodeInfo();

                            steps ++;

                        };

                        cout << "Solution steps " << steps << endl;

                         // Once you're done with the solution you can free the nodes up
                        aStarSearch.FreeSolutionNodes();


                    }
                    else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
                    {
                        cout << "Search terminated. Did not find goal state\n";

                    }

                    // Display the number of loops the search went through
                    cout << "SearchSteps : " << SearchSteps << "\n";

                    SearchCount ++;

                    aStarSearch.EnsureMemoryFreed();
                }

                return 0;
            }
        }
    }

    GameEvent getEvent()
    {
        char c;
        while (std::cin.get(c))
        {
            std::cin.ignore(100, '\n');
            switch (c)
            {
                case 'Q':
                    return GameEvent::quit;
                case 'w':
                    return GameEvent::up;
                case 'a':
                    return GameEvent::left;
                case 's':
                    return GameEvent::down;
                case 'd':
                    return GameEvent::right;
                case 'S':
                    return GameEvent::search;

            }
        }
        return GameEvent::noop;
    }

    void renderHUD()
    {
        std::cout<<"Press: w,a,s,d (to move), S (to search the path) or Q to quit."<<std::endl;
    }


private:
    //Objects
    MapSearchNode* heroStart;
    MapSearchNode* heroEnd;
    PosPlayer* hero_position;
    AStarSearch<MapSearchNode> aStarSearch;

    //Init Functions

    void initGameCharacterNode()
    {
        this->heroStart = new MapSearchNode;
        this->heroEnd = new MapSearchNode;

    }
    void initPosPlayer()
    {
        this->hero_position = new PosPlayer(this->heroStart);
    }

};

// Main

int main(  )
{
    cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";

    // Our sample problem defines the world as a 2d array representing a terrain
    // Each element contains an integer from 0 to 5 which indicates the cost
    // of travel across the terrain. Zero means the least possible difficulty
    // in travelling (think ice rink if you can skate) whilst 5 represents the
    // most difficult. 9 indicates that we cannot pass.

    // Create an instance of the search class...

    sf::RenderWindow window (sf::VideoMode(1920,1080),"Title", sf::Style::Close);

    std::vector<std::vector<sf::RectangleShape>> tileMap;

    tileMap.resize( MAP_WIDTH,std::vector<sf::RectangleShape>());

    for (std::size_t i = 0 ; i<MAP_WIDTH ; i++)
    {
        tileMap[i].resize( MAP_WIDTH ,sf::RectangleShape());
        for (std::size_t j = 0 ; j<MAP_HEIGHT ; j++)
        {
            tileMap[i][j].setSize(sf::Vector2f(100.f,100.f));
            tileMap[i][j].setOutlineThickness(1.f);
            tileMap[i][j].setFillColor(sf::Color::White);
            tileMap[i][j].setOutlineColor(sf::Color::Black);
            tileMap[i][j].setPosition(i * 100.f, j * 100.f);
        }
    }
    while (window.isOpen())
    {
        window.clear();
        for (int i = 0; i <MAP_WIDTH ; i++)
        {
            for (int j = 0; j<MAP_HEIGHT;j++)
            {
                window.draw(tileMap[i][j]);
            }
        }
        window.display();
    }

    /*GameApp game;
    game.renderHUD();
    while (true)
    {
        GameEvent gameEvent = game.getEvent();

        bool quit =game.updateGame(gameEvent);
        if(quit)
        {
            return 0;
        }
    }
     */
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
