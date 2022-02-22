%!octave

function [moveString] = PathMove(move)

	run PathVar;

	switch move
		case PATH_MOVE_DECACC 
			moveString = "Dec/Acc";
		case PATH_MOVE_DECACCSATURATED 
			moveString = "Dec/Acc Saturated";
		case PATH_MOVE_ACCDEC
			moveString = "Acc/Dec";
		case PATH_MOVE_ACCDECSATURATED
			moveString = "Acc/Dec Saturated";
		case PATH_MOVE_DECDEC
			moveString = "Dec/Dec";
		case PATH_MOVE_ACCACC
			moveString = "Acc/Acc";
		otherwise
			moveString = "Unknown";
	end

end % Function
