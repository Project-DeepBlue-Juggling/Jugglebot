# Pattern catalogue

Every juggling Pattern Jugglebot flies, aims at or has on the table carries a catalogue ID **JP-n** (owner decision 2026-10-30). The ID is the handle for conversation, runbooks, plans and map items; the **machine label** is the identifier the code, the admissible box and [the records ledger](records.md) use, and the two never substitute for each other. Rules:

- A JP number is allocated once and never reused or renumbered, even if the Pattern is dropped.
- A Pattern with several machine labels (one ball, fed, phantom) keeps ONE JP number; the labels are listed.
- **Status** is one of *flown* (a records row exists), *destination* (a Pathway's goal), *idea* (a map item), *unfiled* (noted here, no item).
- New Patterns are added here with the next number; the ID goes in the map item's title.

Terms are the glossary's: Site, Apex, Dwell, Beat, Hand, Cup, Columns, Hop, Oval, Ball 1 and Ball 2 ([glossary](glossary.md)).

## One ball

| ID | Name | Machine label | Sites | What moves | Status | Where |
|---|---|---|---|---|---|---|
| JP-1 | Self-throw | `self_toss` | 1 | nothing: throw and catch at one Site | flown | [records](records.md) |
| JP-2 | Hop | `hop` | 2 | the Cup crosses between the Sites once per throw; also the opener of a fed Columns Attempt | flown | [records](records.md) |
| JP-3 | Columns, one ball | `columns_1ball`, `columns_1ball_fed` | 2 | the two-ball Columns motion with one ball real and one phantom | flown | [records](records.md) |
| JP-4 | Oval, one ball | `oval_1ball`, `oval_1ball_fed` | 2 | the caught ball is carried back in the Cup to the throw Site inside the Dwell | flown | [records](records.md) |
| JP-5 | Polygon walk | - | N | one ball hops around N Sites; the smallest Pattern whose Site changes every Beat | idea | [JB-106](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/map/idea-jp-5-polygon-walk-one-ball-n-sites-the-cheapest-way-to-land.md) |
| JP-6 | Magazine throw | `magazine` | 1 | a hand-only rest-to-rest throw at a configured target (a human's hand included); the Platform is stationary | flown | [magazine_throw.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/ros_ws/src/jugglebot/jugglebot/motion/skills/magazine_throw.py) |

## Two balls, one Hand

All two-ball one-Hand Patterns share one timing identity: throws alternate every Beat, each ball's flight plus Dwell is two Beats, and the Cup is empty for half of their difference. With the empty-transit floor near 0.30 s, the Dwell floor near 0.27 s and the Hand's post-release brake capping the Apex near 1.05 m, every one of them lives at an Apex of 0.93 to 1.05 m and a Beat near 0.6 s; they differ only in where the Sites are and whether a ball is carried or aimed.

| ID | Name | Machine label | Sites | What moves | Status | Where |
|---|---|---|---|---|---|---|
| JP-7 | Columns | `columns` | 2 | vertical throws alternate between two fixed Sites | flown | [records](records.md) |
| JP-8 | Oval | - | 2 | each ball is caught at one Site and carried to the other inside the Dwell | destination | [two-ball-oval plan](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/plans/active/two-ball-oval.md) |
| JP-9 | High-low columns | - | 2 | Columns with Ball 1 at Apex 1.05 m and Ball 2 at 0.95 m; the low ball holds longer | idea | [JB-107](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/map/idea-jp-9-high-low-columns-ball-1-at-apex-1-05-m-ball-2-at-0-95-m.md) |
| JP-10 | Turned columns | - | 2 | Columns with the Site pair at a fixed angle about +z (pose planner only) | idea | [JB-108](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/map/idea-jp-10-turned-columns-the-columns-pair-at-fixed-angles-about.md) |
| JP-11 | Rotating columns | - | 2 per Beat | the Site pair turns a few degrees every Beat | idea | [JB-109](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/map/idea-jp-11-rotating-columns-the-pair-turns-a-few-degrees-every.md) |
| JP-12 | Walking columns | - | 2 per Beat | the Site pair translates, across its axis today, along it at the P4 working point | unfiled | this page |
| JP-13 | Rising and sinking columns | - | 2 per Beat | the whole Pattern rides the Platform up and down in z; the planes stay fixed to the Platform centroid | idea | [JB-110](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/map/idea-jp-13-rising-and-sinking-columns-the-whole-pattern-rides-the.md) |
| JP-14 | Centre and orbit | - | 1 + N | Ball 1 in a column at the centre, Ball 2 stepping around a circle of radius about 100 mm | idea | [JB-111](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/map/idea-jp-14-centre-and-orbit-ball-1-in-a-column-ball-2-on-a-circle.md) |
| JP-15 | Over the top | - | 3 | Ball 1 straight up at the centre, Ball 2 arcing over it from side to side; the two-step case of JP-14, needs P4 | idea | [JB-112](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/map/idea-jp-15-over-the-top-ball-1-straight-up-ball-2-arcs-over-it.md) |
| JP-16 | Wide columns | - | 2 | Columns at 175 to 200 mm separation; needs the P4 working point | unfiled | this page |
| JP-18 | Crossing columns | - | 2 | each ball thrown across to the other Site, so the balls cross in the air (a one-Hand shower); not yet probed | unfiled | this page |

## With a human partner

| ID | Name | Machine label | Sites | What moves | Status | Where |
|---|---|---|---|---|---|---|
| JP-17 | Cascade with a partner | - | robot 1, human 1 | a plain three-ball cascade, the owner's hand being the second Hand, the Platform held tilted toward them | idea | [JB-113](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/map/idea-jp-17-three-ball-cascade-with-the-owner-as-the-second-hand.md) |

The Feed (the goal label `reload`, the Ball Butler placing a ball in the Cup) is not a Pattern and has no JP number; the records ledger says the same.
