//
// Created by Lorenzo Ricci on 16/07/2020.
//

#ifndef ELABORATO22_07_2020_SUBJECT_H
#define ELABORATO22_07_2020_SUBJECT_H

#include "Observer.h"


class Subject {
public:
    virtual ~Subject();

    virtual void subscribe (Observer* o)=0;
    virtual void unsubscribe (Observer* o)=0;
    virtual void notify()=0;

};


#endif //ELABORATO22_07_2020_SUBJECT_H
