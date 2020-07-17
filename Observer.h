//
// Created by Lorenzo Ricci on 16/07/2020.
//

#ifndef ELABORATO22_07_2020_OBSERVER_H
#define ELABORATO22_07_2020_OBSERVER_H


class Observer {
public:
    virtual ~Observer();

    virtual void attach() = 0;
    virtual void detach() = 0;

    virtual void update ()=0;

};


#endif //ELABORATO22_07_2020_OBSERVER_H
