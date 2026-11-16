# AGENTS.md â€” Zasady pracy nad kernelem B-Rep

Ten projekt to **czysty, matematyczny kernel 3D B-Rep w jÄ™zyku Rust**.
Nie ma tu UI, renderera, szkicownika ani zaleĹĽnoĹ›ci od aplikacji docelowej.
JedynÄ… miarÄ… poprawnoĹ›ci kodu jest matematyka, niezmienniki topologiczne i testy porĂłwnawcze (Oracle Harness).

## BezwzglÄ™dne zakazy (Anti-Cheat Rules)
1. **Zakaz ukrytej poligonalizacji:** Operacje na Ĺ‚ukach, okrÄ™gach, walcach, stoĹĽkach, sferach, torusach i krzywych NURBS MUSZÄ„ zachowywaÄ‡ dokĹ‚adnÄ… reprezentacjÄ™ analitycznÄ… lub NURBS.
2. **Zakaz `unsafe`, `unwrap()`, `expect()`, `panic!()`, indeksowania `[i]` w kodzie produkcyjnym.**
3. **Zakaz cichych fallbackĂłw:** NieobsĹ‚uĹĽony przypadek brzegowy zwraca `Err(...)`, nigdy uszkodzonÄ… bryĹ‚Ä™.
4. **Zakaz globalnego `EPSILON` w topologii:** Stosujemy Tolerant Modeling (tolerancje per Vertex/Edge) oraz predykaty dokĹ‚adne (`orient2d`, `orient3d`).
5. **Zakaz `Rc<RefCell<...>>`:** Graf B-Rep oparty wyĹ‚Ä…cznie o `SlotMap`.
6. **Zakaz obniĹĽania asercji w testach lub plikach wyroczni.**