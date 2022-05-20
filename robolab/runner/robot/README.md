# Robolab Tools

## Data Structure

The renderer renders a single frame of a single scene per call of `draw_scene`.

A scene is a list of entities.

```
scene = [{ 'id': int, 'part': Part }, { 'id': int, 'part': Part }, ...]
```

where part has the following structure

```
{
    'position' : (float, float, float),
    'orientation' : (float, float, float),
    'scale': (float, float, float),
    'connects_to': [Connection, Connection, ...],
    'drawable': Drawable
}
```

How drawable looks like depends on the implementation (robolab/robolab2)

A connection has the following structure

```
{
    'position' : (float, float, float),
    'orientation' : (float, float, float),
    'axis' : (float, float, float),
    'id': int,
    'connects_to': Part,
}
```

Parts and connections constitute a mutually recursive structure.