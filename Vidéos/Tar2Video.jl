using MeshCat

filename = "First_Step"
TAR_path = joinpath(@__DIR__, "Tar", filename)
MP4_path = joinpath(@__DIR__, "anim", filename)

MeshCat.convert_frames_to_video(TAR_path  * ".tar", MP4_path  * ".mp4")