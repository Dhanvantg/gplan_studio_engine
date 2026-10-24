from operator import imod
from GPLAN.source import inputgraph as inputgraph
from GPLAN.pythongui import gui as gui

# tkinter only needed in GUI paths (gclass is not None); never imported on the API server
class _LazyTk:
    def __getattr__(self, name):
        import tkinter as _tk
        return getattr(_tk, name)
tk = _LazyTk()


class DimParameters:
    __min_width: list
    __max_width: list
    __min_height: list
    __max_height: list
    __min_ratio: list
    __max_ratio: list
    __min_area: list
    __max_area: list
    __plot_height: float
    __plot_width: float
    __symmetric: bool
    __isOptimalEnabled: int
    __isRotationAllowed: int
    __enforce_plot: bool
    __fixed_rooms: list
    def __init__(self, min_width, max_width, plot_height, plot_width,isOptimalEnabled,isRotationAllowed = 1,symmetric = False, min_height = [], max_height = [], min_ratio = [], max_ratio = [], min_area = [], max_area = [], enforce_plot = False, fixed_rooms = None, strict_room_bounds=False, room_nodes=None, use_requested_room_bounds=False):
        self.__isOptimalEnabled = isOptimalEnabled
        self.__isRotationAllowed = isRotationAllowed
        # Hard plot-fit mode (door_connectivity API): plot_width/plot_height
        # hold the caller's REAL footprint (not a slack rejection cap), the
        # boundary pre-selector is skipped so the full topology pool is
        # solved under the cap, and the catalogue is topped up with expanded
        # plans only after every fitting one. Legacy callers (flag absent)
        # keep the old behaviour end to end.
        self.__enforce_plot = bool(enforce_plot)
        self.__min_width = min_width
        self.__max_width = max_width
        self.__plot_height = plot_height
        self.__plot_width = plot_width
        self.__min_height = min_height
        self.__max_height = max_height
        self.__min_ratio = min_ratio
        self.__max_ratio = max_ratio
        # Per-room area rule (sqft). The min-dim SOLVER cannot use area (it
        # reads exactly four lengths per room); these drive the post-solve
        # layers only: _room_size_caps (gap-fill limits), _room_bounds
        # (repair_dimensions bands) and NBC post-processing. Without an
        # explicit max_area the only area cap is max_width * max_height,
        # which is 1.04x-1.42x looser than NBC for every room type.
        self.__min_area = min_area
        self.__max_area = max_area
        # Hard fixed-room constraints used by the min-dimensioned
        # door-connectivity path.  Each entry is normalized by api.py to
        # {room, width, height, anchor, directions}.  Keeping the metadata on
        # DimParameters lets the solver, gap closer and post-processor share
        # one source of truth without changing legacy request shapes.
        self.__fixed_rooms = list(fixed_rooms or [])
        self.strict_room_bounds = bool(strict_room_bounds)
        self.room_nodes = list(room_nodes or [])
        self.use_requested_room_bounds = bool(use_requested_room_bounds)
        # block_checker() splits this on ',', so it has to be a string. The API
        # layer defaults symmetric to the boolean False (views.py, and
        # local_engine_bridge.py), so anything that is not a string is stored as
        # the "()" no-symmetry sentinel that the GUI's Free Dimensions button uses.
        self.__symmetric = symmetric if isinstance(symmetric, str) else "()"


    def get_isOptimalEnabled(self):
        return self.__isOptimalEnabled

    def set_isOptimalEnabled(self, value):
        self.__isOptimalEnabled = value

    def get_isRotationAllowed(self):
        return self.__isRotationAllowed

    def set_isRotationAllowed(self, value):
        self.__isRotationAllowed = value

    def get_enforce_plot(self):
        return self.__enforce_plot

    def set_enforce_plot(self, value):
        self.__enforce_plot = bool(value)

    def get_fixed_rooms(self):
        return self.__fixed_rooms

    def set_fixed_rooms(self, value):
        self.__fixed_rooms = list(value or [])

    def get_min_width(self):
        return self.__min_width

    def set_min_width(self, value):
        self.__min_width = value

    def get_max_width(self):
        return self.__max_width

    def set_max_width(self, value):
        self.__max_width = value

    def get_plot_height(self):
        return self.__plot_height

    def set_plot_height(self, value):
        self.__plot_height = value

    def get_plot_width(self):
        return self.__plot_width

    def set_plot_width(self, value):
        self.__plot_width = value

    def get_min_height(self):
        return self.__min_height

    def set_min_height(self, value):
        self.__min_height = value

    def get_max_height(self):
        return self.__max_height

    def set_max_height(self, value):
        self.__max_height = value

    def get_min_aspect_ratio(self):
        return self.__min_ratio

    def set_min_aspect_ratio(self, value):
        self.__min_ratio = value

    def get_max_aspect_ratio(self):
        return self.__max_ratio

    def set_max_aspect_ratio(self, value):
        self.__max_ratio = value

    def get_min_area(self):
        return self.__min_area

    def set_min_area(self, value):
        self.__min_area = value

    def get_max_area(self):
        return self.__max_area

    def set_max_area(self, value):
        self.__max_area = value

    def get_symmetric(self):
        return self.__symmetric

    def set_symmetric(self, value):
        self.__symmetric = value


class GuiParameters:
    
    __gclass: gui.gui_class = None
    __graph: inputgraph.InputGraph
    __isGui: bool
    __noOfNodes: int #gclass.value[0]
    __edgeCount: int #gclass.value[1]
    __edges: list #gclass.value[2]
    __fptype: str #gclass.value[3]
    __command: str
    __roomNames: list #gclass.value[5]
    __isDimensioned: int #gclass.value[4]
    __isDimensionedCirculation: int #gclass.value[8]
    __isRemoveAddCirculation: int #gclass.value[9]
    __isMinDimensioned: int #gclass.value[10]
    __isPublic: int #gclass.value[11]
    __nodeCoordinates: list #gclass.value[7]
    __roomColors: list #gclass.value[6]
    __letter: str
    __corridor_thickness: float
    __message: str = ""
    __isNonAdj: int #gclass.value[12]
    __isCirculation: int #gclass.value[13]
    __is_multiple_door: bool
    output_data: list = []
    min_dim_inputs: DimParameters = None
    non_adj_list: list = []
    __ptpg_graph: dict = None

    def get_ptpg_graph(self) -> dict:
        return self.__ptpg_graph

    def set_ptpg_graph(self, ptpg_graph: dict):
        self.__ptpg_graph = ptpg_graph
        return self

    def get_min_dim_inputs(self) -> DimParameters:
        return self.min_dim_inputs

    def set_min_dim_inputs(self, min_dim_inputs):
        self.min_dim_inputs = min_dim_inputs
        return self

    def get_fptype(self) -> str:
        return self.__fptype

    def set_fptype(self, fptype: str):
        self.__fptype = fptype
        return self

    def get_corridor_thickness(self) -> float:
        if self.__corridor_thickness is None:
            if self.get_gclass() is not None:
                return self.get_gclass().corridor_thickness
        return self.__corridor_thickness
    
    def set_corridor_thickness(self, corridor_thickness: float):
        self.__corridor_thickness = corridor_thickness
        return self
    
    def get_letter(self) -> str:
        return self.__letter
    
    def set_letter(self, letter: str):
        self.__letter = letter
        return self
    
    def get_isMinDimensioned(self) -> int:
        return self.__isMinDimensioned
    
    def set_isMinDimensioned(self, isMinDimensioned):
        if isMinDimensioned is None:
            self.__isMinDimensioned = 0
        else:
            self.__isMinDimensioned = isMinDimensioned
        return self
        
    
    def get_isCirculation(self) -> int:
        return self.__isCirculation
    
    def set_isCirculation(self, isCirculation):
        if isCirculation is None:
            self.__isCirculation = False
        else:
            self.__isCirculation = isCirculation
        return self
    def get_isPublic(self) -> int:
        return self.__isPublic
    
    def set_isPublic(self, isPublic):
        if isPublic is None:
            self.__isPublic = 0
        else:
            self.__isPublic = isPublic
        return self
        
    def get_isNonAdj(self) -> int:
        return self.__isNonAdj

    def set_isNonAdj(self, isNonAdj: int):
        self.__isNonAdj = isNonAdj
        return self
    
    def get_non_adj_list(self) -> list:
        return self.non_adj_list
    
    def set_non_adj_list(self, non_adj_list: list):
        self.non_adj_list = non_adj_list
        return self
    
    def get_isRemoveAddCirculation(self) -> int:
        return self.__isRemoveAddCirculation
    
    def set_isRemoveAddCirculation(self, isRemoveAddCirculation):
        if isRemoveAddCirculation is None:
            self.isRemoveAddCirculation = 0
        else:
            self.__isRemoveAddCirculation = isRemoveAddCirculation
        return self

    def get_isDimensionedCirculation(self) -> int:
        return self.__isDimensionedCirculation
    
    def set_isDimensionedCirculation(self, isDimensionedCirculation):
        if isDimensionedCirculation is None:
            self.isDimensionedCirculation = 0
        else:
            self.__isDimensionedCirculation = isDimensionedCirculation
        return self

    def get_isDimensioned(self) -> int:
        return self.__isDimensioned
    
    def set_isDimensioned(self, isDimensioned: int):
        if isDimensioned is None:
            self.isDimensioned = 0
        else:
            self.__isDimensioned = isDimensioned 
        return self
    
    def get_nodeCoordinates(self) -> list:
        return self.__nodeCoordinates
    
    def set_nodeCoordinates(self, nodeCoordinates: list):
        self.__nodeCoordinates = nodeCoordinates
        if self.get_gclass() is not None:
            self.get_gclass().coordinates = nodeCoordinates
        return self
         
    def set_gclass(self, gclass: gui.gui_class):
        self.__gclass = gclass
        return self

    def get_isGui(self) -> bool:
        return self.__isGui

    def set_isGui(self, isGui: bool):
        self.__isGui = isGui
        return self

    def get_noOfNodes(self) -> int:
        return self.__noOfNodes

    def set_noOfNodes(self, noOfNodes):
        self.__noOfNodes = noOfNodes
        if self.get_gclass() is not None:
            self.get_gclass().no_of_nodes = noOfNodes
        return self

    def get_edgeCount(self) -> int:
        return self.__edgeCount

    def set_edgeCount(self, edgeCount):
        self.__edgeCount = edgeCount
        if self.get_gclass() is not None:
            self.get_gclass().edge_count = edgeCount
        return self

    def get_edges(self) -> list:
        return self.__edges

    def set_edges(self, edges: list):
        self.__edges = edges
        if self.get_gclass() is not None:
            self.get_gclass().edges = edges
        return self

    def get_command(self) -> str:
        return self.__command

    def set_command(self, command: str):
        self.__command = command
        return self

    def get_roomNames(self) -> list:
        return self.__roomNames

    def set_roomNames(self, roomNames: list):
        self.__roomNames = roomNames
        return self

    def get_roomColors(self) -> list:
        return self.__roomColors

    def set_roomColors(self, roomColors: list):
        self.__roomColors = roomColors
        return self
    
    def get_gclass(self) -> gui.gui_class:
        return self.__gclass
    
    def get_graph(self) -> inputgraph.InputGraph:
        return self.__graph
    
    def set_graph(self, graph: inputgraph.InputGraph):
        self.__graph = graph
        return self
    
    def set_message(self, message: str):
        if self.get_message() is not None:
            self.__message += message
        return self.__message
    
    def get_message(self) -> str:
        return self.__message

    def get_dim_constraints(self):
        if self.get_gclass() is not None:
            return self.get_gclass().dimensional_constraints
        return None

    def get_is_multiple_door(self):
        return self.__is_multiple_door
    
    def set_is_multiple_door(self, is_multiple_door):
        self.__is_multiple_door = is_multiple_door
        return self


    def __init__(self, gclass = None, graph = None):
        if gclass is not None:
            self.set_gclass(gclass)
            self.set_noOfNodes(gclass.value[0])
            self.set_edgeCount(gclass.value[1])
            self.set_edges(gclass.value[2])
            self.set_fptype(gclass.value[3])
            self.set_isDimensioned(gclass.value[4])
            self.set_roomNames(gclass.value[5])
            self.set_roomColors(gclass.value[6])
            self.set_nodeCoordinates(gclass.value[7])
            self.set_isDimensionedCirculation(gclass.value[8])
            self.set_isRemoveAddCirculation(gclass.value[9])
            self.set_isMinDimensioned(gclass.value[10])
            self.set_isPublic(gclass.value[11])
            self.set_isNonAdj(gclass.value[12])
            self.set_isCirculation(gclass.value[13])
            self.set_isGui(True)
            self.set_letter(gclass.letter)
            self.set_corridor_thickness(gclass.corridor_thickness)
            self.set_message("")
            self.set_command(gclass.command)
            self.set_is_multiple_door(False)#Change this later on to take input from GUI

            self.output_data = []
        
        if graph is not None:
            self.set_graph(graph)
            self.set_edgeCount(graph.edgecnt)
            self.set_noOfNodes(graph.nodecnt)
            # self.set_edges(graph.edges)
            self.set_nodeCoordinates(graph.coordinates)
            self.set_isGui(False)
    
    def print_gui(self, string):
        self.set_message(string)
        if self.get_gclass() is not None:
            self.get_gclass().textbox.insert(tk.END, string) # type: ignore
            self.get_gclass().textbox.insert(tk.END, "\n") # type: ignore
    def get_output_data(self):
        if self.get_gclass() is not None:
            return self.get_gclass().output_data
        return self.output_data

    def _append_output_data(self, graph_new):
        import math
        import numpy as np
        
        if self.get_gclass() is not None:
            self.get_gclass().output_data.append(graph_new)
        else:
            self.output_data.append(graph_new)

        def find_areas(graph_new):
            for shape in graph_new.final_traversal:
                coords = np.array(shape)
                x = coords[:, 0]
                y = coords[:, 1]
                area = 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
                graph_new.area.append(area)

        if graph_new is not None and len(graph_new.final_traversal)==0: #FOR SANITY
            graph_new.final_traversal = inputgraph.get_final_traversal(graph_new)
        print("Final appending data",graph_new.final_traversal)
        # graph_new.room_name_coords()
        if(len(graph_new.area) < graph_new.nodecnt):
            find_areas(graph_new)
        origin = {'x': 0, 'y': 0}
        max_area=np.amax(graph_new.area)
        # plot_area=np.sum(graph_new.area)
        scale2=(math.exp(-0.3*math.sqrt(max_area)+math.log(0.8))+0.1)
        scale = 1
        #Add room coordinates to final output data
        shapes = graph_new.final_traversal

        for i in range(len(shapes)):
            width=shapes[i][3][0] - shapes[i][0][0]
            coordi = (((2 * shapes[i][0][0] ) * scale / 2) + origin['x'] + 0.5*scale2,((shapes[i][0][1] + shapes[i][1][1]) * scale / 2) + origin['y'])
            graph_new.name_coords.append(coordi)


    def _set_output_data(self, output_data):
        if self.get_gclass() is not None:
            self.get_gclass().output_data = output_data
        else:
            self.output_data = output_data
    
    def _set_time_taken(self, time_taken):
        if self.get_gclass() is not None:
            self.get_gclass().time_taken = time_taken
    
    def _set_num_rfp(self, num_rfp):
        if self.get_gclass() is not None:
            self.get_gclass().num_rfp = num_rfp
            
    def _set_pdf_colors(self, pdf_colors):
        if self.get_gclass() is not None:
            self.get_gclass().pdf_colors = pdf_colors
            
    def _set_output_found(self, output_found):
        if self.get_gclass() is not None:
            self.get_gclass().output_found = output_found
            
    def _set_multiple_output_found(self, multiple_output_found):
        if self.get_gclass() is not None:
            self.get_gclass().multiple_output_found = multiple_output_found
                    
    def _set_dim_constraints(self, dim_constraints):
        if self.get_gclass() is not None:
            self.get_gclass().dimensional_constraints = dim_constraints

    def _set_ptpg(self, graph):
        if self.get_gclass() is not None:
            self.get_gclass().ptpg = graph

