(define (domain uav)
    (:types 
        uav - object
        waypoint - object
        route - object
    )
    
    (:predicates  
        (at ?x0 - uav ?y0 - waypoint)
        (taken_off ?x - uav)
        (landed ?x - uav)
        (visited ?x - uav ?y - waypoint)
        (next ?y1 - waypoint ?y2 - waypoint ?r - route)
        (current-waypoint ?x - uav ?y - waypoint ?r - route)
        (completed-route ?x - uav ?r - route)
    )

    (:action fly
    :parameters (?x - uav ?y - waypoint ?z - waypoint) 
    :precondition (and 
                   (at ?x ?y)
                   (taken_off ?x)
                   (or
                       ; Free flight
                       (and 
                            (not (exists (?r - route ?wp1 - waypoint) (next ?y ?wp1 ?r)))
                            (not (exists (?r - route ?wp2 - waypoint) (next ?wp2 ?z ?r)))
                       )
                       ; Starting or following a route
                       (exists (?r - route) 
                           (and 
                                (next ?y ?z ?r)
                                (or 
                                    (current-waypoint ?x ?y ?r)
                                    (and 
                                        (not (exists (?w - waypoint) (next ?w ?y ?r)))
                                        (not (exists (?wp - waypoint) (current-waypoint ?x ?wp ?r)))
                                    )
                                )
                           )
                       )
                   )
    )
    :effect (and 
             (not (at ?x ?y))
             (at ?x ?z)
             (forall (?r - route)
                     (when (next ?y ?z ?r)
                           (and 
                                (not (current-waypoint ?x ?y ?r))
                                (current-waypoint ?x ?z ?r)
                           )
                     )
             )
             (visited ?x ?z)
             (forall (?r - route)
                     (when (and (next ?y ?z ?r) (not (exists (?w - waypoint) (next ?z ?w ?r))))
                           (completed-route ?x ?r)
                     )
             )
        )
    )




    (:action take_off
        :parameters (?x - uav)
        :precondition (landed ?x)
        :effect (and 
                 (taken_off ?x) 
                 (not (landed ?x)))
    )

    (:action land
        :parameters (?x - uav)
        :precondition (taken_off ?x)
        :effect (and 
                 (landed ?x) 
                 (not (taken_off ?x)))
    )
)
