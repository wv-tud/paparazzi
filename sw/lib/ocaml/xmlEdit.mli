(*
 * $Id$
 *
 * XML editor
 *  
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 *)

(** XML types base on th xml-light library *)

type t
(** The whole XML data structure *)

type node
(** One data structure node. Warning: it is not an absolute
node designation: it may not remain valid after strucure modifications
(reordering, deletion addition, ... *)

type tag = string
type attributes = (string * string) list

type event = Deleted | Modified of attributes | New_child of node

val create : Dtd.dtd -> Xml.xml -> (t * GWindow.window)
(** [create dtd xml] Opens a display of [xml] with contextual right button
actions constrained by [dtd]. Returns the corresponding model. *)

val xml_of_node : node -> Xml.xml
val xml_of_view : t -> Xml.xml
(** [xml_of_view v] Returns the XML displayed data structure *)

val root : t -> node

val child : node -> tag -> node
val tag : node -> string
val attribs : node -> attributes
val attrib : node -> string -> string (* No case match *)
val children : node -> node list
(** Xml-light like acces functions *)

val set_attribs : node -> attributes -> unit
val delete : node -> unit
val add_child : node -> tag -> attributes -> node
(** Modifications *)

val connect : node -> (event -> unit) -> unit
(** To be kept informed about modifications *)
