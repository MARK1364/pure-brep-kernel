# TEST_PLAN.md â€” Weryfikacja i Multi-Kernel Oracle

- Po kaĹĽdej operacji: `validate_solid()` (Euler-PoincarĂ© $V - E + F - R = 2(S - G)$, 2-rozmaitoĹ›Ä‡, spĂłjnoĹ›Ä‡ `3D <-> S(pcurve)`, objÄ™toĹ›Ä‡ > 0).
- Wyrocznie JSON (`TestCase` -> `OracleReport`): porĂłwnanie Ours vs OCCT (osobny proces Python `cadquery-ocp`), Truck (osobna binarka Rust) oraz NVIDIA SMLib (osobna binarka C++ z `usd-brep`).
- Torture Suite: wspĂłĹ‚pĹ‚aszczyznowoĹ›Ä‡, styk krawÄ™dzi/wierzchoĹ‚kĂłw, walec-walec o rĂłwnych promieniach (Ăłsemka), zmiany genusu, fuzzing izometryczny.