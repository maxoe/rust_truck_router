# Getting started with Rust

## Installieren über Rustup

[`rustup`](https://rustup.rs/) ist ein Installer der einem das Management unterschiedlicher Compilerversionen (aktuell halten, mehrere Versionen parallel, Extra-Komponenten verwalten, etc.) abnimmt.
Rustup kann entweder über das Packetmanagement eurer Distribution oder die Anweisungen auf der Website installiert werden.
Die Website-Methode sollte auch verwendet werden, wenn Rust auf den Poolrechnern/Computes verwendet werden soll.
Diese installiert Rustup im Homeverzeichnis des aktuellen Nutzers.
Dokumentation gibt es [hier](https://github.com/rust-lang-nursery/rustup.rs/blob/master/README.md);

Den aktuellen stable Compiler könnt ihr dann mit `rustup install stable` installieren.
Seine Toolchains kann man mit `rustup update` gesammelt auf den aktuellen Stand bringen.

## Rust lernen

Die Rust-Community legt Wert auf gute Dokumentation.
Dementsprechend sind Libraries (insbesondere die Standardlibrary) in der Regel gut dokumentiert.
Außerdem gibt es umfangreiche [Einstiegsressourcen](https://www.rust-lang.org/en-US/documentation.html).
Insbesondere mit ["Dem Buch"](https://doc.rust-lang.org/book/second-edition/index.html) kriegt man kompakt und gut sortiert einmal alle Sprachkonzepte erklärt.

## Cargo: Buildsystem & Dependency Management

Rust hat ein Build- und Dependency Managementsystem. Yay.
Heißt `cargo`.
Mit `cargo build` kann man Programme bauen.
Im Falle dieses Repos baut das nur die Basislib und noch keine Executable, dazu dann z.B. `cargo build --bin example` oder statt example ein anderes Programm wie `decode_vector`.
Mit `cargo run` kann man bauen und direkt ausführen.
Argumente an das Programm kommen nach einem `--`.
Sobald die Performance interessant ist unbedingt `cargo run --release` nutzen - das ist ungefähr was `-O3` bei C++ ist.
Alles zusammen `cargo run --release --bin example -- /path/to/graph/files/dir/`.

Mit `cargo check` kann man das Programm checken ohne zu bauen, das kann einem einiges an Zeit sparen.

## Clippy

Rust hat einen exzellenten Linter, der einem sehr hilft idiomatischen und performanten Code zu schreiben.
Das ist insbesondere wenn man noch nicht viel Erfahrung mit der Sprache hat extrem sinnvoll!
Installieren kann man Clippy mit `rustup component add clippy-preview`.
Anstatt `cargo check` ruft man dann `cargo clippy` auf und kann sich auf viel hilfreiches Feedback freuen.

# Rust Routenplanungs-Basis-Framework

In diesem GIT-Repository finden Sie eine kleine Codebasis die Sie zur Bearbeitung des Übungsblatts verwenden sollen.
Sie haben Zugriff auf vier Module und 3 Hilfsprogramme.

In `types` werden Gewichts und ID Typen definiert sowie eine wichtige Gewichtskonstante: `INFINITY`.
`INFINITY` repräsentiert die unendliche Distanz und ist so gewählt, dass die Konstante verdoppelt werden kann, ohne einen Überlauf zu verursachen, d.h., der Ausdruck `INFINITY < INFINITY + INFINITY` ist unproblematisch.
Im Modul `time` gibt es zwei Funktionen die verwendet werden können um die Laufzeit von Code zu messen.
Das Modul `index_heap` enthält eine Prioritätswarteschlange (`std::collections::BinaryHeap` ist problematisch für unseren Anwendungsfall, da keine `decrease_key` Operation vorhanden).
Das `io` Modul dienen dem Einlesen und der Ausgabe von Daten.
Jede Datendatei ist das binäre Abbild eines `std::vector`s im Speicher, d.h., ein Vektor von 100 `u32`s wird in einer Datei gespeichert die genau 400 Byte lang (Wir gehen stets davon aus, dass ein int 32 Bit hat.) ist.
Die entsprechenden Funktionen sind über Traits definiert und können so direkt auf den entsprechenden Objekten aufgerufen werden.
Das kann z.B. so aussehen:

```Rust
extern crate stud_rust_base;
use stud_rust_base::io::*;

let head = Vec::<u32>::load_from("head_file_name").expect("could not read head");
let lat = Vec::<f32>::load_from("node_latitude_file_name").expect("could not read lat");
head.write_to("output_file").expect("could not write head");
```

Die Dateien in `src/bin/` sind einmal ein Beispielprogramm sowieso Hilfsprogramme.
`encode_vector` und `decode_vector` konvertieren Vektoren von und zu textuellen Darstellungen.
Das Programm `compare_vector` vergleicht ob zwei Vektoren identisch sind und wenn sie es nicht sind gibt es eine Übersicht über die Unterschiede.
Fügen sie Ihre Programme in `src/bin/` hinzu, diese werden dann von `cargo` automatisch gefunden.

## Docs

`cargo doc --open` öffnet die Dokumentation zu dem bereitgestelltem Code.

## Graphen

Knoten und Kanten werden durch numerische IDs identifiziert, die von `0` bis `n-1` bzw. `m-1` gehen, wobei `n` die Anzahl an Knoten und `m` die Anzahl an gerichteten Kanten ist.
Wir speichern gewichtete und gerichtete Graphen in einer Ajdazenzarraydarstellung.
Ein gerichteter und gewichteter Graph besteht aus 3 Vektoren.
Diese heißen `first_out`, `head` und `weight`.
Um über die ausgehenden Kanten eines Knoten zu iterieren können Sie den folgenden Code verwenden:

```Rust
extern crate stud_rust_base;
use stud_rust_base::{types::*, io::*};

let first_out = Vec::<EdgeId>::load_from("first_out_file_name").expect("could not read first_out");
let head = Vec::<NodeId>::load_from("head_file_name").expect("could not read head");
let travel_time = Vec::<Weight>::load_from("weight_file_name").expect("could not read travel_time");

let node_id = 42;
for edge_id in first_out[node_id] .. first_out[node_id + 1] {
    println!("There is an arc from {} to {} with weight {}", node_id, head[edge_id as usize], travel_time[edge_id as usize]);
}
```

**Hinweis**: `head` und `weight` haben so viel Elemente wie es Kanten gibt.
`first_out` hat ein Element mehr als es Knoten gibt.
Das erste Element von `first_out` ist immer 0 und das letzte ist die Anzahl an Kanten.
Für alle Graphen gibt es zwei unterschiedliche Kantengewichte: Reisezeit und Reiselänge.
Des Weiteren gibt es für manche Graphen zusätzliche für jeden Knoten die geographische Position.
Diese wird als zwei `float` Vektoren abgespeichert die für jeden Knoten den Längen- und Breitengrad angeben.

Im Verzeichnis `/algoDaten/praktikum/graph` liegen die Daten von mehreren Graphen in diesem Format.
Manche dienen nur zu Testzwecken während andere zur Aufgabenbewertung verwendet werden.
Die Testgraphen entsprechen ganz grob Stupferich, Karlsruhe\&Umgebung, Deutschland\&Umgebung und (West-)Europa.
Die Aufgabengraphen haben die Größe des Deutschlandgraphen.

**Achtung**: Der Europagraph könnte zu groß sein für den Hauptspeicher von manchen Poolraumrechnern.

## Hinweise zur Nutzung im Routenplanungspraktikum

Der Quellcode soll durch das Ausführen von `cargo build --all` mit dem aktuellen stabilen Compiler (1.29.2) übersetzt werden können.
Auf den Poolraumrechner ist kein Rust Compiler vorinstalliert.
Sie können aber für ihren Nutzer lokal `rustup` und damit dann einen aktuellen Compiler installieren.
Die Nutzung von nicht stabilen nightly Features ist nicht erlaubt.
Das verwenden externer crates ist nicht erlaubt.
Die Rust-Standardbibliothek ist nicht extern.
