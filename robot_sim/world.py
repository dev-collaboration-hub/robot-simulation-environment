"""M13 deterministic 2D world scenes with validated JSON save/load and reset.

Objects and obstacles remain separate. All geometry is in metres on the same XY
plane as ArmModel. This is a scene manager, not a general 3D rigid-body world.
"""
from __future__ import annotations
from dataclasses import dataclass, replace
import json
from pathlib import Path
from .math3d import Vec2, finite
from .collision import CircleObstacle, BoxObstacle, point_box_distance


@dataclass(frozen=True)
class WorldObject:
    name: str
    position: Vec2
    radius: float = .06

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('object name required')
        if not isinstance(self.position, Vec2):
            raise TypeError('object position must be Vec2')
        r = finite(self.radius, 'object radius')
        if r <= 0:
            raise ValueError('radius must be positive')
        object.__setattr__(self, 'radius', r)


@dataclass(frozen=True)
class Scene:
    name: str
    objects: tuple[WorldObject, ...]
    obstacles: tuple[CircleObstacle | BoxObstacle, ...] = ()
    bounds: tuple[float, float, float, float] = (-3., -3., 3., 3.)

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('scene name required')
        object.__setattr__(self, 'objects', tuple(self.objects))
        object.__setattr__(self, 'obstacles', tuple(self.obstacles))
        if len(set(x.name for x in self.objects)) != len(self.objects):
            raise ValueError('duplicate object names')
        if not all(isinstance(x, WorldObject) for x in self.objects):
            raise TypeError('expected WorldObject instances')
        if not all(isinstance(o, (CircleObstacle, BoxObstacle)) for o in self.obstacles):
            raise TypeError('unsupported obstacle shape')
        if len(self.bounds) != 4:
            raise ValueError('scene bounds must be xmin,ymin,xmax,ymax')
        bounds = tuple(finite(n, 'bounds') for n in self.bounds)
        x1,y1,x2,y2=bounds
        if x1 >= x2 or y1 >= y2:
            raise ValueError('invalid world bounds')
        object.__setattr__(self, 'bounds', bounds)
        for obj in self.objects:
            if not self.contains(obj.position, obj.radius):
                raise ValueError('object outside scene bounds')
            if self.obstructed(obj.position, obj.radius):
                raise ValueError('object starts intersecting an obstacle')

    def contains(self, position: Vec2, radius: float = 0.) -> bool:
        x1,y1,x2,y2 = self.bounds
        return (x1+radius <= position.x <= x2-radius and
                y1+radius <= position.y <= y2-radius)

    def obstructed(self, position: Vec2, radius: float = 0.) -> bool:
        for o in self.obstacles:
            if isinstance(o, CircleObstacle):
                if position.distance_to(Vec2(o.x,o.y)) < radius+o.radius-1e-10:
                    return True
            elif point_box_distance(position, o) < radius-1e-10:
                return True
        return False

    @classmethod
    def from_dict(cls, raw):
        if not isinstance(raw, dict) or set(raw)-{'name','bounds','objects','obstacles'}:
            raise ValueError('scene must be object with known keys')
        if not isinstance(raw.get('objects'), list) or not isinstance(raw.get('obstacles',[]), list):
            raise ValueError('objects and obstacles must be arrays')
        objects = []
        for entry in raw['objects']:
            if not isinstance(entry, dict) or set(entry)-{'name','position','radius'}:
                raise ValueError('invalid object entry')
            p = entry.get('position')
            if not isinstance(p,list) or len(p)!=2:
                raise ValueError('object position must be [x,y]')
            objects.append(WorldObject(entry['name'], Vec2(*p), entry.get('radius',.06)))
        obstacles=[]
        for entry in raw.get('obstacles',[]):
            if not isinstance(entry,dict) or set(entry)-{'type','x','y','radius','min_x','min_y','max_x','max_y'}:
                raise ValueError('invalid obstacle entry')
            kind=entry.get('type')
            if kind=='circle':
                if set(entry)!={'type','x','y','radius'}:
                    raise ValueError('circle requires x,y,radius')
                obstacles.append(CircleObstacle(entry['x'],entry['y'],entry['radius']))
            elif kind=='box':
                if set(entry)!={'type','min_x','min_y','max_x','max_y'}:
                    raise ValueError('box requires bounds')
                obstacles.append(BoxObstacle(entry['min_x'],entry['min_y'],entry['max_x'],entry['max_y']))
            else:
                raise ValueError('unsupported obstacle type')
        return cls(raw.get('name','scene'), tuple(objects), tuple(obstacles),
                   tuple(raw.get('bounds', [-3.,-3.,3.,3.])))

    def to_dict(self):
        objects=[{'name':o.name,'position':[o.position.x,o.position.y],'radius':o.radius}
                 for o in self.objects]
        obstacles=[]
        for o in self.obstacles:
            if isinstance(o,CircleObstacle):
                obstacles.append({'type':'circle','x':o.x,'y':o.y,'radius':o.radius})
            else:
                obstacles.append({'type':'box','min_x':o.min_x,'min_y':o.min_y,
                                  'max_x':o.max_x,'max_y':o.max_y})
        return {'name':self.name,'bounds':list(self.bounds),'objects':objects,'obstacles':obstacles}

    @classmethod
    def load(cls, path: str | Path):
        with open(path,encoding='utf-8') as f:
            return cls.from_dict(json.load(f))

    def save(self, path: str | Path):
        with open(path,'w',encoding='utf-8') as f:
            json.dump(self.to_dict(),f,indent=2,sort_keys=True)
            f.write('\n')


class World:
    def __init__(self, scene: Scene):
        self.scene=scene
        self.reset()

    def reset(self):
        self.objects={o.name:o for o in self.scene.objects}
        self.ticks=0

    def get(self, name: str) -> WorldObject:
        return self.objects[name]

    def can_place(self, name: str, position: Vec2) -> bool:
        obj=self.get(name)
        if not self.scene.contains(position,obj.radius) or self.scene.obstructed(position,obj.radius):
            return False
        return all(position.distance_to(o.position) >= obj.radius+o.radius-1e-10
                   for k,o in self.objects.items() if k != name)

    def move(self,name: str,position: Vec2,carried: bool=False) -> bool:
        if not isinstance(position,Vec2):
            raise TypeError('position must be Vec2')
        # Carried objects remain in the arena and may not intersect obstacles
        # or other blocks; unsafe moves are refused without changing state.
        if not self.can_place(name,position):
            return False
        self.objects[name]=replace(self.get(name),position=position)
        return True

    def positions(self):
        return {name:(obj.position.x,obj.position.y) for name,obj in sorted(self.objects.items())}

    def as_scene(self):
        return replace(self.scene,objects=tuple(self.objects[k] for k in sorted(self.objects)))


def example_scene(name: str = 'pick_and_place') -> Scene:
    if name=='pick_and_place':
        return Scene('pick_and_place',(WorldObject('block',Vec2(1.4,.5)),))
    if name=='empty':
        return Scene('empty',())
    if name=='obstacles':
        return Scene('obstacles',(WorldObject('block',Vec2(1.5,-.3)),),
                     (CircleObstacle(.7,.8,.15),BoxObstacle(-1.6,-.3,-1.1,.4)))
    if name=='target_reaching':
        return Scene('target_reaching',(WorldObject('marker',Vec2(1.2,.9),.05),))
    raise ValueError('unknown example scene')
