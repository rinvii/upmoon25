import { useEffect, useRef, useState } from 'react'
import type { CameraStream, StreamStatus } from '../bridge/types'

const RECONNECT_MS = 500

const cameraSocketNames: Record<CameraStream['id'], string> = {
  front: 'rgb',
  rear: 'rear',
}

export function useCameraFrame(camera: CameraStream, wsBase: string | undefined) {
  const imageRef = useRef<HTMLImageElement | null>(null)
  const [hasFrame, setHasFrame] = useState(false)
  const [socketState, setSocketState] = useState<StreamStatus>('unknown')

  useEffect(() => {
    if (!wsBase) {
      return
    }

    let closed = false
    let reconnectTimer: ReturnType<typeof setTimeout> | undefined
    let socket: WebSocket | undefined
    let currentUrl: string | null = null
    let pendingUrl: string | null = null
    let frameFlushRaf: number | null = null
    let live = false
    let hasAnyFrame = false

    function revokeCurrent() {
      if (currentUrl) {
        URL.revokeObjectURL(currentUrl)
        currentUrl = null
      }
    }

    function clearImage() {
      if (imageRef.current) {
        imageRef.current.removeAttribute('src')
      }
      if (frameFlushRaf !== null) {
        window.cancelAnimationFrame(frameFlushRaf)
        frameFlushRaf = null
      }
      if (pendingUrl) {
        URL.revokeObjectURL(pendingUrl)
        pendingUrl = null
      }
      revokeCurrent()
      live = false
      hasAnyFrame = false
      setSocketState('missing')
      setHasFrame(false)
    }

    function publishFrame(nextUrl: string) {
      if (closed) {
        URL.revokeObjectURL(nextUrl)
        return
      }
      // Latest-frame wins: keep only the newest pending frame.
      if (pendingUrl) URL.revokeObjectURL(pendingUrl)
      pendingUrl = nextUrl
      if (frameFlushRaf !== null) return
      frameFlushRaf = window.requestAnimationFrame(() => {
        frameFlushRaf = null
        if (closed || !pendingUrl) return
        const frameUrl = pendingUrl
        pendingUrl = null
        if (imageRef.current) imageRef.current.src = frameUrl
        if (!live) {
          live = true
          setSocketState('live')
        }
        if (!hasAnyFrame) {
          hasAnyFrame = true
          setHasFrame(true)
        }
        revokeCurrent()
        currentUrl = frameUrl
      })
    }

    function connect() {
      if (closed) return
      setSocketState((prev) => (prev === 'live' ? prev : 'connecting'))
      const path = cameraSocketNames[camera.id]
      socket = new WebSocket(`${wsBase}/camera/ws/${path}`)
      socket.binaryType = 'arraybuffer'

      socket.onmessage = (event) => {
        if (closed) return
        const blob = event.data instanceof Blob
          ? event.data
          : new Blob([event.data as ArrayBuffer], { type: 'image/jpeg' })
        publishFrame(URL.createObjectURL(blob))
      }

      socket.onerror = () => {
        if (!closed) setSocketState('stale')
      }

      socket.onclose = () => {
        if (closed) return
        setSocketState('missing')
        clearImage()
        reconnectTimer = window.setTimeout(connect, RECONNECT_MS)
      }
    }

    connect()

    return () => {
      closed = true
      if (reconnectTimer !== undefined) window.clearTimeout(reconnectTimer)
      socket?.close()
      clearImage()
    }
  }, [camera.id, wsBase])

  const effectiveHasFrame = wsBase ? hasFrame : false
  const effectiveSocketState: StreamStatus = wsBase ? socketState : 'unknown'

  return { imageRef, hasFrame: effectiveHasFrame, socketState: effectiveSocketState, configured: Boolean(wsBase) }
}
