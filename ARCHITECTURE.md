# ARCHITECTURE.md â€” Czysta Architektura Kernela B-Rep

1. `crates/brep-math`: wektory, AABB/BVH, dokĹ‚adne predykaty (`orient2d`/`orient3d`), funkcje bazowe NURBS.
2. `crates/brep-geom`: krzywe 3D (`Line`, `Circle`, `Nurbs`), `PCurve2D`, powierzchnie (`Plane`, `Cylinder`, docelowo `Cone`/`Sphere`/`Torus`/`NurbsSurface`), dane kanoniczne pod przyszĹ‚y Solver 3D i PMI.
3. `crates/brep-topo`: arena B-Rep (`SlotMap`), `OriginTag` pod Persistent Naming/PMI, Ĺ›cisĹ‚y walidator `validate_solid`.
4. `crates/brep-ops`: prymitywy, generator profili (`Extrude`/`Revolve`), Booleany 3D, `Chamfer`/`Fillet`, `Shell`/`Healing`.
5. `crates/oracle-harness`: izolowany runner JSON do testĂłw porĂłwnawczych (Ours vs OCCT vs Truck vs NVIDIA SMLib). Ĺ»adna paczka `brep-*` nie moĹĽe zaleĹĽeÄ‡ od wyroczni.