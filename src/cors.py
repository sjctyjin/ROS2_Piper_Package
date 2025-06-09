from aiohttp import web
import aiohttp_cors

async def handle(request):
    filename = request.match_info.get('filename', "index.html")
    return web.FileResponse(filename)

app = web.Application()
cors = aiohttp_cors.setup(app, defaults={
    "*": aiohttp_cors.ResourceOptions(
        allow_credentials=True,
        expose_headers="*",
        allow_headers="*",
    )
})

resource = cors.add(app.router.add_resource("/{filename:.*}"))
cors.add(resource.add_route("GET", handle))

web.run_app(app, port=8000)
